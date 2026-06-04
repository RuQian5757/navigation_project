#!/usr/bin/env python3
"""
Realtime 3D pointcloud viewer using PyVista.
Connects to a local TCP pointcloud bridge at localhost:9000 and renders live 3D points.
"""
import argparse
import socket
import struct
import threading
import time

import numpy as np

try:
    import pyvista as pv
except ImportError:
    print('PyVista not found. Install with: pip install pyvista')
    raise


def parse_args():
    parser = argparse.ArgumentParser(description='Realtime 3D pointcloud viewer using PyVista')
    parser.add_argument('--host', default='127.0.0.1', help='TCP bridge host')
    parser.add_argument('--port', default=9000, type=int, help='TCP bridge port')
    parser.add_argument('--point-size', default=4, type=int, help='Point size')
    parser.add_argument('--cmap', default='viridis', help='Colormap for point coloring')
    parser.add_argument('--update-rate', default=20.0, type=float, help='Max refresh rate (Hz)')
    return parser.parse_args()


class RealtimePointCloudViewer:
    def __init__(self, host, port, point_size=4, cmap='viridis', update_rate=20.0):
        self.host = host
        self.port = port
        self.point_size = point_size
        self.cmap = cmap
        self.update_interval = 1.0 / max(update_rate, 1.0)
        self.sock = None
        self.lock = threading.Lock()
        self.latest = None
        self.running = True
        self.frame_count = 0
        self.first_frame = True

        self.cloud = pv.PolyData(np.zeros((1, 3), dtype=np.float32))
        self.cloud.point_data['elevation'] = np.zeros(1, dtype=np.float32)

        self.plotter = pv.Plotter(title='Live PointCloud 3D', window_size=(1280, 768))
        self.actor = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(1.0)
        self.sock.connect((self.host, self.port))
        print(f'✓ Connected to {self.host}:{self.port}')

    def recv_all(self, size):
        data = bytearray()
        while len(data) < size and self.running:
            try:
                chunk = self.sock.recv(size - len(data))
                if not chunk:
                    return None
                data.extend(chunk)
            except socket.timeout:
                continue
        return data

    def recv_loop(self):
        try:
            while self.running:
                header = self.recv_all(4)
                if header is None:
                    break
                (count,) = struct.unpack('!I', header)
                if count <= 0:
                    continue
                payload = self.recv_all(count * 3 * 4)
                if payload is None:
                    break
                points = np.frombuffer(payload, dtype=np.float32).reshape(-1, 3)
                with self.lock:
                    self.latest = points.astype(np.float64)
                    self.frame_count += 1
                    if self.frame_count == 1:
                        print(f'✓ Received first packet: {len(points)} points')
        except Exception as exc:
            print('Recv thread error:', exc)
        finally:
            self.running = False
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass

    def setup_plotter(self):
        self.plotter.set_background('black')
        self.plotter.add_text('Realtime 3D Point Cloud', font_size=16, color='white', position='upper_left')
        self.actor = self.plotter.add_points(
            self.cloud,
            point_size=self.point_size,
            scalars='elevation',
            cmap=self.cmap,
            render_points_as_spheres=True,
            opacity=0.9,
            show_scalar_bar=False,
        )
        self.plotter.add_scalar_bar(title='Elevation', n_labels=5, label_font_size=12)

    def update_cloud(self, points):
        new_cloud = pv.PolyData(points.astype(np.float32))
        new_cloud.point_data['elevation'] = points[:, 2].astype(np.float32)
        if self.actor is not None:
            self.plotter.remove_actor(self.actor, reset_camera=False, render=False)
        self.actor = self.plotter.add_points(
            new_cloud,
            point_size=self.point_size,
            scalars='elevation',
            cmap=self.cmap,
            render_points_as_spheres=True,
            opacity=0.9,
            show_scalar_bar=False,
        )
        self.cloud = new_cloud

    def run(self):
        try:
            self.connect()
        except Exception as exc:
            print(f'✗ Unable to connect: {exc}')
            print('請先啟動 pointcloud_bridge')
            return

        thread = threading.Thread(target=self.recv_loop, daemon=True)
        thread.start()

        self.setup_plotter()
        self.plotter.show(auto_close=False, interactive_update=True)

        last_update = 0.0
        try:
            while self.running:
                points = None
                with self.lock:
                    if self.latest is not None:
                        points = self.latest.copy()

                if points is not None and points.size:
                    now = time.time()
                    if now - last_update >= self.update_interval:
                        self.update_cloud(points)
                        if self.first_frame:
                            self.plotter.reset_camera()
                            self.first_frame = False
                        self.plotter.title = f'Realtime 3D Point Cloud (frame={self.frame_count}, points={len(points)})'
                        self.plotter.render()
                        last_update = now

                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            thread.join(timeout=2.0)
            self.plotter.close()


if __name__ == '__main__':
    args = parse_args()
    viewer = RealtimePointCloudViewer(args.host, args.port, point_size=args.point_size, cmap=args.cmap, update_rate=args.update_rate)
    viewer.run()
