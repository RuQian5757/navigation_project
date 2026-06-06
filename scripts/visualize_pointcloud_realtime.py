#!/usr/bin/env python3
"""
Realtime 3D point cloud viewer for DynamicWorldCloud.

Subscribes directly to Gazebo Transport topic /world/dynamic_cloud.
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

pv = None


def parse_args():
    parser = argparse.ArgumentParser(description='Realtime DynamicWorldCloud 3D viewer')
    parser.add_argument('--topic', default='/world/dynamic_cloud',
                        help='Gazebo Transport PointCloudPacked topic')
    parser.add_argument('--partition', default='dynamic_cloud_test',
                        help='Gazebo Transport partition; must match scripts/run_gazebo.sh')
    parser.add_argument('--point-size', default=3, type=int, help='Rendered point size')
    parser.add_argument('--cmap', default='viridis', help='Colormap for z elevation')
    parser.add_argument('--update-rate', default=20.0, type=float,
                        help='Max viewer refresh rate in Hz')
    parser.add_argument('--max-render-points', default=250000, type=int,
                        help='Downsample very large clouds for rendering only')
    parser.add_argument('--window-width', default=1280, type=int)
    parser.add_argument('--window-height', default=768, type=int)
    return parser.parse_args()


class LatestCloudBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.points = None
        self.frame = 0
        self.source_points = 0
        self.updated_at = 0.0
        self.status = 'waiting'

    def set_points(self, points, source_points=None):
        with self.lock:
            self.points = points
            self.frame += 1
            self.source_points = int(source_points if source_points is not None else len(points))
            self.updated_at = time.time()
            self.status = 'streaming'

    def snapshot(self):
        with self.lock:
            if self.points is None:
                return None, self.frame, self.source_points, self.updated_at, self.status
            return self.points.copy(), self.frame, self.source_points, self.updated_at, self.status

    def set_status(self, status):
        with self.lock:
            self.status = status


def downsample_points(points, max_points):
    if max_points <= 0 or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def parse_pointcloud_packed(msg):
    point_count = int(msg.width) * int(msg.height)
    if point_count <= 0 or msg.point_step <= 0:
        return np.empty((0, 3), dtype=np.float32)

    offsets = {}
    datatypes = {}
    for field in msg.field:
        offsets[field.name] = int(field.offset)
        datatypes[field.name] = int(field.datatype)

    float32_type = 6
    if 'xyz' in offsets:
        if datatypes.get('xyz') != float32_type:
            raise ValueError('Only FLOAT32 xyz fields are supported')
        xyz_offset = offsets['xyz']
        axis_offsets = [xyz_offset, xyz_offset + 4, xyz_offset + 8]
    else:
        if not all(axis in offsets for axis in ('x', 'y', 'z')):
            raise ValueError('PointCloudPacked does not contain xyz or x/y/z fields')
        if any(datatypes.get(axis) != float32_type for axis in ('x', 'y', 'z')):
            raise ValueError('Only FLOAT32 x/y/z fields are supported')
        axis_offsets = [offsets['x'], offsets['y'], offsets['z']]

    required = point_count * int(msg.point_step)
    if len(msg.data) < required:
        raise ValueError('PointCloudPacked data is shorter than width*height*point_step')

    raw = np.frombuffer(msg.data, dtype=np.uint8, count=required)
    rows = raw.reshape(point_count, int(msg.point_step))
    points = np.empty((point_count, 3), dtype=np.float32)
    for column, start in enumerate(axis_offsets):
        axis_bytes = rows[:, start:start + 4].copy()
        points[:, column] = axis_bytes.view(np.float32).reshape(-1)

    finite = np.isfinite(points).all(axis=1)
    return points[finite]


class GazeboPointCloudSubscriber:
    def __init__(self, topic, partition, buffer, max_render_points):
        self.topic = topic
        self.partition = partition
        self.buffer = buffer
        self.max_render_points = max_render_points
        self.node = None

    def start(self):
        os.environ['GZ_PARTITION'] = self.partition
        try:
            import gz.transport13 as gz_transport
            from gz.msgs10 import pointcloud_packed_pb2
        except ModuleNotFoundError:
            system_dist = '/usr/lib/python3/dist-packages'
            if system_dist not in sys.path:
                sys.path.append(system_dist)
            import gz.transport13 as gz_transport
            from gz.msgs10 import pointcloud_packed_pb2

        self.node = gz_transport.Node()

        def callback(msg):
            try:
                points = parse_pointcloud_packed(msg)
                rendered = downsample_points(points, self.max_render_points)
                self.buffer.set_points(rendered, source_points=len(points))
            except Exception as exc:
                self.buffer.set_status(f'parse error: {exc}')
                print(f'PointCloudPacked parse error: {exc}')

        self.node.subscribe(pointcloud_packed_pb2.PointCloudPacked, self.topic, callback)
        self.buffer.set_status(f'subscribed: {self.topic}')
        print(f'✓ Subscribed to {self.topic} on GZ_PARTITION={self.partition}')


class RealtimePointCloudViewer:
    def __init__(self, args):
        self.args = args
        self.buffer = LatestCloudBuffer()
        self.update_interval = 1.0 / max(args.update_rate, 1.0)
        self.last_frame_rendered = -1
        self.first_frame = True
        self.running = True

        self.cloud = pv.PolyData(np.zeros((1, 3), dtype=np.float32))
        self.cloud.point_data['elevation'] = np.zeros(1, dtype=np.float32)
        self.plotter = pv.Plotter(
            title='DynamicWorldCloud Viewer',
            window_size=(args.window_width, args.window_height),
        )
        self.actor = None
        self.text_actor = None
        self.subscriber = None

    def create_subscriber(self):
        return GazeboPointCloudSubscriber(
            self.args.topic,
            self.args.partition,
            self.buffer,
            self.args.max_render_points,
        )

    def setup_plotter(self):
        self.plotter.set_background('#05070a')
        self.text_actor = self.plotter.add_text(
            'Waiting for point cloud...',
            font_size=14,
            color='white',
            position='upper_left',
        )
        self.actor = self.plotter.add_points(
            self.cloud,
            point_size=self.args.point_size,
            scalars='elevation',
            cmap=self.args.cmap,
            render_points_as_spheres=True,
            opacity=0.95,
            show_scalar_bar=False,
        )
        self.plotter.add_axes()
        self.plotter.add_scalar_bar(title='Z', n_labels=5, label_font_size=10)

    def update_cloud(self, points):
        if points.size == 0:
            return
        rendered = points.astype(np.float32, copy=False)
        elevation = rendered[:, 2].astype(np.float32, copy=False)

        if self.cloud.n_points == len(rendered):
            self.cloud.points = rendered
            self.cloud.point_data['elevation'] = elevation
            self.cloud.Modified()
            return

        self.cloud = pv.PolyData(rendered)
        self.cloud.point_data['elevation'] = elevation
        if self.actor is not None:
            self.plotter.remove_actor(self.actor, reset_camera=False, render=False)
        self.actor = self.plotter.add_points(
            self.cloud,
            point_size=self.args.point_size,
            scalars='elevation',
            cmap=self.args.cmap,
            render_points_as_spheres=True,
            opacity=0.95,
            show_scalar_bar=False,
        )

    def update_status_text(self, frame, rendered_points, source_points, updated_at, status, points):
        if points is not None and points.size:
            mins = points.min(axis=0)
            maxs = points.max(axis=0)
            age = time.time() - updated_at if updated_at else 0.0
            text = (
                f'DynamicWorldCloud | frame={frame} | rendered={rendered_points} | '
                f'source={source_points} | age={age:.2f}s\n'
                f'x=[{mins[0]:.2f}, {maxs[0]:.2f}] '
                f'y=[{mins[1]:.2f}, {maxs[1]:.2f}] '
                f'z=[{mins[2]:.2f}, {maxs[2]:.2f}]'
            )
        else:
            text = f'DynamicWorldCloud | {status}'
        self.plotter.remove_actor(self.text_actor, render=False)
        self.text_actor = self.plotter.add_text(
            text,
            font_size=14,
            color='white',
            position='upper_left',
        )

    def run(self):
        self.subscriber = self.create_subscriber()
        self.subscriber.start()
        self.setup_plotter()
        self.plotter.show(auto_close=False, interactive_update=True)

        last_update = 0.0
        try:
            while self.running:
                points, frame, source_points, updated_at, status = self.buffer.snapshot()
                now = time.time()
                if now - last_update >= self.update_interval:
                    if points is not None and frame != self.last_frame_rendered:
                        self.update_cloud(points)
                        self.last_frame_rendered = frame
                        if self.first_frame:
                            self.plotter.reset_camera()
                            self.first_frame = False
                    rendered_count = 0 if points is None else len(points)
                    self.update_status_text(
                        frame, rendered_count, source_points, updated_at, status, points
                    )
                    self.plotter.render()
                    last_update = now
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if hasattr(self.subscriber, 'stop'):
                self.subscriber.stop()
            self.plotter.close()


if __name__ == '__main__':
    args = parse_args()
    try:
        import pyvista as _pyvista
    except ImportError:
        print('PyVista not found. Install with: pip install pyvista')
        raise
    pv = _pyvista
    viewer = RealtimePointCloudViewer(args)
    viewer.run()
