#!/usr/bin/env python3
"""Train Random Forest models for Octree leaf voxel semantics.

The script reads every CSV in ./data whose filename starts with ``train_`` by
default, trains:

- a RandomForestClassifier for ``label`` (0=free, 1=obstacle, 2=stair)
- a RandomForestRegressor for ``obstacle_probability``

The exported model bundle can later be used to predict labels for feature CSVs
that were not generated with the ``train_`` prefix.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


LABEL_COLUMN = "label"
PROBABILITY_COLUMN = "obstacle_probability"
DEFAULT_MODEL_OUTPUT = Path("models/random_forest_voxel_model.pkl")
DEFAULT_REPORT_OUTPUT = Path("models/random_forest_voxel_report.json")
DEFAULT_CPP_MODEL_OUTPUT = Path("models/random_forest_voxel_model.rf.txt")

# Identifiers are useful for tracing a leaf back to Gazebo / Octree, but they
# can make the model memorize a map instead of learning geometric semantics.
IDENTIFIER_COLUMNS = {
    "node_index",
    "entity_id",
    "morton_code",
    "morton_low_8bits",
}


class TrainingDataError(RuntimeError):
    """Raised when the available training data is not usable."""


class DependencyError(RuntimeError):
    """Raised when Python ML dependencies are missing."""


def load_ml_dependencies() -> None:
    global np
    global RandomForestClassifier
    global RandomForestRegressor
    global accuracy_score
    global classification_report
    global confusion_matrix
    global mean_absolute_error
    global mean_squared_error
    global r2_score
    global train_test_split

    try:
        import numpy as _np
        from sklearn.ensemble import (
            RandomForestClassifier as _RandomForestClassifier,
            RandomForestRegressor as _RandomForestRegressor,
        )
        from sklearn.metrics import (
            accuracy_score as _accuracy_score,
            classification_report as _classification_report,
            confusion_matrix as _confusion_matrix,
            mean_absolute_error as _mean_absolute_error,
            mean_squared_error as _mean_squared_error,
            r2_score as _r2_score,
        )
        from sklearn.model_selection import train_test_split as _train_test_split
    except ImportError as exc:
        raise DependencyError(
            "Missing Python ML dependencies. Install them with: "
            "python3 -m pip install numpy scikit-learn"
        ) from exc

    np = _np
    RandomForestClassifier = _RandomForestClassifier
    RandomForestRegressor = _RandomForestRegressor
    accuracy_score = _accuracy_score
    classification_report = _classification_report
    confusion_matrix = _confusion_matrix
    mean_absolute_error = _mean_absolute_error
    mean_squared_error = _mean_squared_error
    r2_score = _r2_score
    train_test_split = _train_test_split


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train Random Forest models from data/train_*.csv files."
    )
    parser.add_argument(
        "--data-dir",
        default="data",
        type=Path,
        help="Directory containing training CSV files, default ./data.",
    )
    parser.add_argument(
        "--pattern",
        default="train_*.csv",
        help="Glob pattern inside --data-dir, default train_*.csv.",
    )
    parser.add_argument(
        "--model-output",
        default=DEFAULT_MODEL_OUTPUT,
        type=Path,
        help=f"Output pickle model path, default {DEFAULT_MODEL_OUTPUT}.",
    )
    parser.add_argument(
        "--report-output",
        default=DEFAULT_REPORT_OUTPUT,
        type=Path,
        help=f"Output JSON report path, default {DEFAULT_REPORT_OUTPUT}.",
    )
    parser.add_argument(
        "--cpp-model-output",
        default=DEFAULT_CPP_MODEL_OUTPUT,
        type=Path,
        help=f"Output C++ readable Random Forest text model, default {DEFAULT_CPP_MODEL_OUTPUT}.",
    )
    parser.add_argument(
        "--test-size",
        default=0.2,
        type=float,
        help="Validation split ratio, default 0.2.",
    )
    parser.add_argument(
        "--n-estimators",
        default=300,
        type=int,
        help="Number of trees for each Random Forest, default 300.",
    )
    parser.add_argument(
        "--random-state",
        default=42,
        type=int,
        help="Random seed, default 42.",
    )
    parser.add_argument(
        "--min-samples",
        default=30,
        type=int,
        help="Minimum total rows required for training, default 30.",
    )
    parser.add_argument(
        "--min-samples-per-class",
        default=2,
        type=int,
        help="Minimum rows required for each label class, default 2.",
    )
    parser.add_argument(
        "--include-identifiers",
        action="store_true",
        help="Allow node_index/entity_id/morton_code features. Usually avoid this.",
    )
    parser.add_argument(
        "--predict",
        type=Path,
        action="append",
        help="Optional feature CSV to predict after training. Can be passed more than once.",
    )
    parser.add_argument(
        "--predict-pattern",
        default="*.csv",
        help="Auto-predict CSV glob inside --data-dir, default *.csv.",
    )
    parser.add_argument(
        "--no-auto-predict",
        action="store_true",
        help="Do not auto-predict non-train CSV files from --data-dir.",
    )
    parser.add_argument(
        "--prediction-output",
        type=Path,
        help="Output CSV for a single --predict file. Default prefixes filename with predicted_.",
    )
    parser.add_argument(
        "--prediction-output-dir",
        type=Path,
        help="Output directory for auto or multiple predictions. Default keeps each source directory.",
    )
    return parser.parse_args()


def discover_training_files(data_dir: Path, pattern: str) -> list[Path]:
    files = sorted(data_dir.glob(pattern))
    if not files:
        raise TrainingDataError(
            f"No training CSV files found: {data_dir / pattern}. "
            "Generate files with FEATURE_TRAIN=1 ./scripts/run_feature_export.sh"
        )
    return files


def discover_prediction_files(
    data_dir: Path,
    predict_pattern: str,
    training_files: list[Path],
    explicit_predict_files: list[Path] | None,
    auto_predict: bool,
) -> list[Path]:
    if explicit_predict_files:
        return explicit_predict_files
    if not auto_predict:
        return []

    training_paths = {path.resolve() for path in training_files}
    candidates = []
    for path in sorted(data_dir.glob(predict_pattern)):
        if not path.is_file() or path.suffix.lower() != ".csv":
            continue
        if path.resolve() in training_paths:
            continue
        if path.name.startswith("train_") or path.name.startswith("predicted_"):
            continue
        candidates.append(path)
    return candidates


def is_finite_float(value: str) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def choose_feature_columns(fieldnames: Iterable[str], include_identifiers: bool) -> list[str]:
    ignored = {LABEL_COLUMN, PROBABILITY_COLUMN}
    if not include_identifiers:
        ignored |= IDENTIFIER_COLUMNS
    return [name for name in fieldnames if name not in ignored]


def load_training_rows(
    paths: list[Path],
    include_identifiers: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str], dict[str, Any]]:
    feature_columns: list[str] | None = None
    x_rows: list[list[float]] = []
    labels: list[int] = []
    probabilities: list[float] = []
    skipped_rows = 0
    loaded_rows_by_file: dict[str, int] = {}

    for path in paths:
        with path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                raise TrainingDataError(f"{path} has no CSV header.")
            required = {LABEL_COLUMN, PROBABILITY_COLUMN}
            missing = required - set(reader.fieldnames)
            if missing:
                raise TrainingDataError(f"{path} is missing required columns: {sorted(missing)}")

            if feature_columns is None:
                feature_columns = choose_feature_columns(reader.fieldnames, include_identifiers)
                if not feature_columns:
                    raise TrainingDataError("No usable feature columns found.")
            else:
                missing_features = set(feature_columns) - set(reader.fieldnames)
                if missing_features:
                    raise TrainingDataError(
                        f"{path} is missing feature columns used by earlier files: "
                        f"{sorted(missing_features)}"
                    )

            file_rows = 0
            for row in reader:
                values: list[float] = []
                try:
                    label = int(float(row[LABEL_COLUMN]))
                    probability = float(row[PROBABILITY_COLUMN])
                    if label not in (0, 1, 2) or not math.isfinite(probability):
                        raise ValueError
                    for column in feature_columns:
                        raw = row.get(column, "")
                        if not is_finite_float(raw):
                            raise ValueError
                        values.append(float(raw))
                except (TypeError, ValueError):
                    skipped_rows += 1
                    continue

                x_rows.append(values)
                labels.append(label)
                probabilities.append(min(1.0, max(0.0, probability)))
                file_rows += 1

            loaded_rows_by_file[str(path)] = file_rows

    assert feature_columns is not None
    metadata = {
        "loaded_rows_by_file": loaded_rows_by_file,
        "skipped_rows": skipped_rows,
    }
    return (
        np.asarray(x_rows, dtype=np.float32),
        np.asarray(labels, dtype=np.int64),
        np.asarray(probabilities, dtype=np.float32),
        feature_columns,
        metadata,
    )


def validate_training_data(
    x: np.ndarray,
    y_label: np.ndarray,
    min_samples: int,
    min_samples_per_class: int,
) -> Counter:
    if x.shape[0] < min_samples:
        raise TrainingDataError(
            f"Only {x.shape[0]} usable rows were found; at least {min_samples} are required. "
            "Please generate more train_ CSV data."
        )

    label_counts = Counter(int(v) for v in y_label)
    if len(label_counts) < 2:
        raise TrainingDataError(
            f"Only one label class is present: {dict(label_counts)}. "
            "Random Forest classification needs at least two classes."
        )

    weak_classes = {
        label: count for label, count in label_counts.items()
        if count < min_samples_per_class
    }
    if weak_classes:
        raise TrainingDataError(
            f"Some label classes have too few rows: {weak_classes}. "
            f"Need at least {min_samples_per_class} per class."
        )
    return label_counts


def split_data(
    x: np.ndarray,
    y_label: np.ndarray,
    y_probability: np.ndarray,
    test_size: float,
    random_state: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if not 0.0 < test_size < 0.5:
        raise TrainingDataError("--test-size must be > 0 and < 0.5")
    return train_test_split(
        x,
        y_label,
        y_probability,
        test_size=test_size,
        random_state=random_state,
        stratify=y_label,
    )


def train_models(
    x_train: np.ndarray,
    y_label_train: np.ndarray,
    y_probability_train: np.ndarray,
    n_estimators: int,
    random_state: int,
) -> tuple[RandomForestClassifier, RandomForestRegressor]:
    classifier = RandomForestClassifier(
        n_estimators=n_estimators,
        random_state=random_state,
        n_jobs=-1,
        class_weight="balanced_subsample",
        min_samples_leaf=2,
    )
    regressor = RandomForestRegressor(
        n_estimators=n_estimators,
        random_state=random_state,
        n_jobs=-1,
        min_samples_leaf=2,
    )
    classifier.fit(x_train, y_label_train)
    regressor.fit(x_train, y_probability_train)
    return classifier, regressor


def train_final_models(
    x: np.ndarray,
    y_label: np.ndarray,
    y_probability: np.ndarray,
    n_estimators: int,
    random_state: int,
) -> tuple[RandomForestClassifier, RandomForestRegressor]:
    return train_models(
        x,
        y_label,
        y_probability,
        n_estimators=n_estimators,
        random_state=random_state,
    )


def evaluate_models(
    classifier: RandomForestClassifier,
    regressor: RandomForestRegressor,
    x_test: np.ndarray,
    y_label_test: np.ndarray,
    y_probability_test: np.ndarray,
) -> dict[str, Any]:
    label_pred = classifier.predict(x_test)
    probability_pred = np.clip(regressor.predict(x_test), 0.0, 1.0)
    mse = mean_squared_error(y_probability_test, probability_pred)
    return {
        "label_accuracy": float(accuracy_score(y_label_test, label_pred)),
        "classification_report": classification_report(
            y_label_test,
            label_pred,
            labels=[0, 1, 2],
            target_names=["free", "obstacle", "stair"],
            zero_division=0,
            output_dict=True,
        ),
        "confusion_matrix_labels": [0, 1, 2],
        "confusion_matrix": confusion_matrix(
            y_label_test,
            label_pred,
            labels=[0, 1, 2],
        ).tolist(),
        "obstacle_probability_mae": float(mean_absolute_error(y_probability_test, probability_pred)),
        "obstacle_probability_rmse": float(math.sqrt(mse)),
        "obstacle_probability_r2": float(r2_score(y_probability_test, probability_pred)),
    }


def feature_importances(
    model: RandomForestClassifier | RandomForestRegressor,
    feature_columns: list[str],
    limit: int = 15,
) -> list[dict[str, float | str]]:
    pairs = sorted(
        zip(feature_columns, model.feature_importances_),
        key=lambda item: item[1],
        reverse=True,
    )
    return [
        {"feature": feature, "importance": float(importance)}
        for feature, importance in pairs[:limit]
    ]


def export_cpp_forest_model(
    classifier: RandomForestClassifier,
    regressor: RandomForestRegressor,
    feature_columns: list[str],
    output_path: Path,
) -> None:
    """Export sklearn forests to a small line-based format for C++17 inference."""
    class_index_by_label = {
        int(label): index for index, label in enumerate(classifier.classes_)
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("RFVOXEL_TEXT 1\n")
        handle.write(f"FEATURE_COUNT {len(feature_columns)}\n")
        for feature in feature_columns:
            handle.write(f"FEATURE {feature}\n")

        handle.write("CLASS_COUNT 3\n")
        handle.write("CLASSES 0 1 2\n")
        handle.write(f"CLASSIFIER_TREES {len(classifier.estimators_)}\n")
        for estimator in classifier.estimators_:
            tree = estimator.tree_
            handle.write(f"TREE {tree.node_count}\n")
            for node_index in range(tree.node_count):
                values = [0.0, 0.0, 0.0]
                raw_values = tree.value[node_index][0]
                for label in (0, 1, 2):
                    class_index = class_index_by_label.get(label)
                    if class_index is not None:
                        values[label] = float(raw_values[class_index])
                handle.write(
                    "NODE "
                    f"{int(tree.children_left[node_index])} "
                    f"{int(tree.children_right[node_index])} "
                    f"{int(tree.feature[node_index])} "
                    f"{float(tree.threshold[node_index]):.17g} "
                    f"{values[0]:.17g} {values[1]:.17g} {values[2]:.17g}\n"
                )

        handle.write(f"REGRESSOR_TREES {len(regressor.estimators_)}\n")
        for estimator in regressor.estimators_:
            tree = estimator.tree_
            handle.write(f"TREE {tree.node_count}\n")
            for node_index in range(tree.node_count):
                value = float(tree.value[node_index][0][0])
                handle.write(
                    "NODE "
                    f"{int(tree.children_left[node_index])} "
                    f"{int(tree.children_right[node_index])} "
                    f"{int(tree.feature[node_index])} "
                    f"{float(tree.threshold[node_index]):.17g} "
                    f"{value:.17g}\n"
                )
        handle.write("END\n")


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [to_jsonable(v) for v in value]
    if isinstance(value, tuple):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def evaluate_prediction_ground_truth(
    y_label_true: list[int],
    y_label_pred: list[int],
    y_probability_true: list[float],
    y_probability_pred: list[float],
) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    if y_label_true:
        metrics["label_accuracy"] = float(accuracy_score(y_label_true, y_label_pred))
        metrics["classification_report"] = classification_report(
            y_label_true,
            y_label_pred,
            labels=[0, 1, 2],
            target_names=["free", "obstacle", "stair"],
            zero_division=0,
            output_dict=True,
        )
        metrics["confusion_matrix_labels"] = [0, 1, 2]
        metrics["confusion_matrix"] = confusion_matrix(
            y_label_true,
            y_label_pred,
            labels=[0, 1, 2],
        ).tolist()
    if y_probability_true:
        mse = mean_squared_error(y_probability_true, y_probability_pred)
        metrics["obstacle_probability_mae"] = float(
            mean_absolute_error(y_probability_true, y_probability_pred)
        )
        metrics["obstacle_probability_rmse"] = float(math.sqrt(mse))
        metrics["obstacle_probability_r2"] = float(
            r2_score(y_probability_true, y_probability_pred)
        )
    return metrics


def prediction_output_for(input_path: Path, output_dir: Path | None = None) -> Path:
    parent = output_dir if output_dir is not None else input_path.parent
    return parent / f"predicted_{input_path.name}"


def predict_csv(
    classifier: RandomForestClassifier,
    regressor: RandomForestRegressor,
    feature_columns: list[str],
    input_path: Path,
    output_path: Path,
) -> dict[str, Any]:

    with input_path.open(newline="") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames:
            raise TrainingDataError(f"{input_path} has no CSV header.")
        missing = set(feature_columns) - set(reader.fieldnames)
        if missing:
            raise TrainingDataError(
                f"{input_path} is missing required feature columns: {sorted(missing)}"
            )

        fieldnames = list(reader.fieldnames)
        for column in ("predicted_label", "predicted_obstacle_probability"):
            if column not in fieldnames:
                fieldnames.append(column)

        rows = list(reader)
        feature_rows: list[list[float]] = []
        valid_indices: list[int] = []
        skipped_predictions = 0
        for index, row in enumerate(rows):
            try:
                feature_rows.append([float(row[column]) for column in feature_columns])
                valid_indices.append(index)
            except (TypeError, ValueError):
                row["predicted_label"] = ""
                row["predicted_obstacle_probability"] = ""
                skipped_predictions += 1

        y_label_true: list[int] = []
        y_label_pred: list[int] = []
        y_probability_true: list[float] = []
        y_probability_pred: list[float] = []

        if feature_rows:
            features = np.asarray(feature_rows, dtype=np.float32)
            predicted_labels = classifier.predict(features)
            predicted_probabilities = np.clip(regressor.predict(features), 0.0, 1.0)
            for valid_index, pred_label_raw, pred_probability_raw in zip(
                valid_indices,
                predicted_labels,
                predicted_probabilities,
            ):
                row = rows[valid_index]
                pred_label = int(pred_label_raw)
                pred_probability = float(pred_probability_raw)
                row["predicted_label"] = str(pred_label)
                row["predicted_obstacle_probability"] = f"{pred_probability:.6f}"

                if LABEL_COLUMN in row and is_finite_float(row[LABEL_COLUMN]):
                    label_true = int(float(row[LABEL_COLUMN]))
                    if label_true in (0, 1, 2):
                        y_label_true.append(label_true)
                        y_label_pred.append(pred_label)
                if PROBABILITY_COLUMN in row and is_finite_float(row[PROBABILITY_COLUMN]):
                    y_probability_true.append(
                        min(1.0, max(0.0, float(row[PROBABILITY_COLUMN])))
                    )
                    y_probability_pred.append(pred_probability)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="") as target:
            writer = csv.DictWriter(target, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    return {
        "input": str(input_path),
        "output": str(output_path),
        "rows": len(rows),
        "skipped_predictions": skipped_predictions,
        "ground_truth_rows": len(y_label_true),
        "metrics": evaluate_prediction_ground_truth(
            y_label_true,
            y_label_pred,
            y_probability_true,
            y_probability_pred,
        ),
    }


def main() -> int:
    args = parse_args()

    try:
        load_ml_dependencies()

        train_files = discover_training_files(args.data_dir, args.pattern)
        x, y_label, y_probability, feature_columns, load_metadata = load_training_rows(
            train_files,
            include_identifiers=args.include_identifiers,
        )
        label_counts = validate_training_data(
            x,
            y_label,
            min_samples=args.min_samples,
            min_samples_per_class=args.min_samples_per_class,
        )

        split = split_data(
            x,
            y_label,
            y_probability,
            test_size=args.test_size,
            random_state=args.random_state,
        )
        x_train, x_test, y_label_train, y_label_test, y_prob_train, y_prob_test = split
        validation_classifier, validation_regressor = train_models(
            x_train,
            y_label_train,
            y_prob_train,
            n_estimators=max(1, args.n_estimators),
            random_state=args.random_state,
        )
        metrics = evaluate_models(
            validation_classifier,
            validation_regressor,
            x_test,
            y_label_test,
            y_prob_test,
        )

        final_classifier, final_regressor = train_final_models(
            x,
            y_label,
            y_probability,
            n_estimators=max(1, args.n_estimators),
            random_state=args.random_state,
        )

        prediction_files = discover_prediction_files(
            args.data_dir,
            args.predict_pattern,
            train_files,
            args.predict,
            auto_predict=not args.no_auto_predict,
        )
        if args.prediction_output and len(prediction_files) != 1:
            raise TrainingDataError(
                "--prediction-output can only be used when exactly one prediction CSV is selected."
            )

        prediction_reports = []
        for prediction_file in prediction_files:
            if args.prediction_output:
                prediction_output = args.prediction_output
            else:
                prediction_output = prediction_output_for(
                    prediction_file,
                    output_dir=args.prediction_output_dir,
                )
            prediction_reports.append(
                predict_csv(
                    final_classifier,
                    final_regressor,
                    feature_columns,
                    prediction_file,
                    prediction_output,
                )
            )

        metadata: dict[str, Any] = {
            "trained_at": datetime.now(timezone.utc).isoformat(),
            "train_files": [str(path) for path in train_files],
            "prediction_files": [str(path) for path in prediction_files],
            "model_output": str(args.model_output),
            "cpp_model_output": str(args.cpp_model_output),
            "rows_total": int(x.shape[0]),
            "rows_train": int(x_train.shape[0]),
            "rows_test": int(x_test.shape[0]),
            "skipped_rows": load_metadata["skipped_rows"],
            "loaded_rows_by_file": load_metadata["loaded_rows_by_file"],
            "label_counts": dict(label_counts),
            "feature_columns": feature_columns,
            "include_identifiers": bool(args.include_identifiers),
            "n_estimators": int(args.n_estimators),
            "random_state": int(args.random_state),
            "validation_metrics": metrics,
            "prediction_reports": prediction_reports,
            "top_label_features": feature_importances(final_classifier, feature_columns),
            "top_probability_features": feature_importances(final_regressor, feature_columns),
        }

        bundle = {
            "label_model": final_classifier,
            "obstacle_probability_model": final_regressor,
            "feature_columns": feature_columns,
            "metadata": metadata,
        }

        args.model_output.parent.mkdir(parents=True, exist_ok=True)
        with args.model_output.open("wb") as handle:
            pickle.dump(bundle, handle)
        export_cpp_forest_model(
            final_classifier,
            final_regressor,
            feature_columns,
            args.cpp_model_output,
        )
        args.report_output.parent.mkdir(parents=True, exist_ok=True)
        args.report_output.write_text(
            json.dumps(to_jsonable(metadata), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        print("[train_model] training complete")
        print(f"  files: {len(train_files)}")
        print(f"  rows: {x.shape[0]} usable, {load_metadata['skipped_rows']} skipped")
        print(f"  labels: {dict(label_counts)}")
        print(f"  features: {len(feature_columns)}")
        print(f"  label_accuracy: {metrics['label_accuracy']:.4f}")
        print(f"  obstacle_probability_mae: {metrics['obstacle_probability_mae']:.4f}")
        print(f"  obstacle_probability_rmse: {metrics['obstacle_probability_rmse']:.4f}")
        print(f"  model: {args.model_output}")
        print(f"  cpp_model: {args.cpp_model_output}")
        print(f"  report: {args.report_output}")
        if prediction_reports:
            for report in prediction_reports:
                metrics_text = ""
                if report["metrics"]:
                    label_acc = report["metrics"].get("label_accuracy")
                    prob_mae = report["metrics"].get("obstacle_probability_mae")
                    parts = []
                    if label_acc is not None:
                        parts.append(f"label_accuracy={label_acc:.4f}")
                    if prob_mae is not None:
                        parts.append(f"prob_mae={prob_mae:.4f}")
                    metrics_text = " (" + ", ".join(parts) + ")" if parts else ""
                print(
                    "  predictions: "
                    f"{report['rows']} rows -> {report['output']}{metrics_text}"
                )
        else:
            print("  predictions: no non-train CSV selected")

    except DependencyError as exc:
        print(f"[train_model] {exc}", file=sys.stderr)
        return 2
    except TrainingDataError as exc:
        print(f"[train_model] Training data is not sufficient: {exc}", file=sys.stderr)
        return 1
    except FileNotFoundError as exc:
        print(f"[train_model] File not found: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
