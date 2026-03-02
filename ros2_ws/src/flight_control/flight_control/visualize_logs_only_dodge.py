import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from tensorboard.backend.event_processing import event_accumulator


TARGET_TAGS = [
    "rollout/ep_rew_mean",
    "train/explained_variance",
    "train/entropy_loss",
    "train/value_loss",
    "train/std",
]


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) < window:
        return values
    kernel = np.ones(window, dtype=float) / float(window)
    smoothed = np.convolve(values, kernel, mode="valid")
    pad_left = np.full(window - 1, smoothed[0], dtype=float)
    return np.concatenate([pad_left, smoothed])


def find_event_files(log_root: Path) -> List[Path]:
    return sorted(log_root.rglob("events.out.tfevents.*"))


def load_scalars(event_file: Path) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    ea = event_accumulator.EventAccumulator(
        str(event_file),
        size_guidance={
            event_accumulator.SCALARS: 0,
            event_accumulator.HISTOGRAMS: 0,
            event_accumulator.IMAGES: 0,
            event_accumulator.AUDIO: 0,
            event_accumulator.TENSORS: 0,
        },
    )
    ea.Reload()

    scalar_data: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for tag in ea.Tags().get("scalars", []):
        events = ea.Scalars(tag)
        if not events:
            continue
        steps = np.array([ev.step for ev in events], dtype=float)
        values = np.array([ev.value for ev in events], dtype=float)
        scalar_data[tag] = (steps, values)

    return scalar_data


def plot_metrics(
    all_runs: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]],
    out_dir: Path,
    smooth_window: int,
    show_plot: bool,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    all_found_tags = sorted({tag for run_data in all_runs.values() for tag in run_data.keys()})
    if not all_found_tags:
        print("[WARN] 찾은 scalar tag가 없습니다.")
        return

    print("\n[INFO] 발견된 scalar tags:")
    for tag in all_found_tags:
        print(f"  - {tag}")

    print("\n[INFO] 요청된 대상 tags:")
    for tag in TARGET_TAGS:
        print(f"  - {tag}")

    for tag in TARGET_TAGS:
        tag_exists = any(tag in run_data for run_data in all_runs.values())
        if not tag_exists:
            print(f"[WARN] tag 없음: {tag}")
            continue

        plt.figure(figsize=(10, 5))
        has_data = False

        for run_name, run_data in all_runs.items():
            if tag not in run_data:
                continue

            steps, values = run_data[tag]
            if len(steps) == 0:
                continue

            smoothed = moving_average(values, smooth_window)
            plt.plot(steps, smoothed, label=run_name)
            has_data = True

        if not has_data:
            plt.close()
            continue

        plt.title(tag)
        plt.xlabel("Step")
        plt.ylabel("Value")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()

        safe_tag = tag.replace("/", "_").replace(" ", "_")
        save_path = out_dir / f"{safe_tag}.png"
        plt.savefig(save_path, dpi=140)
        print(f"[SAVE] {save_path}")

        if show_plot:
            plt.show()
        else:
            plt.close()


def build_run_name(log_root: Path, event_file: Path) -> str:
    parent = event_file.parent
    try:
        rel = parent.relative_to(log_root)
        return str(rel)
    except ValueError:
        return parent.name


def main() -> None:
    default_root = Path(__file__).resolve().parent / "logs_only_dodge"

    parser = argparse.ArgumentParser(
        description="logs_only_dodge TensorBoard 이벤트 파일을 읽어 스칼라 그래프를 생성합니다."
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=default_root,
        help=f"로그 루트 디렉터리 (기본값: {default_root})",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="그래프 저장 경로 (기본값: <log-root>/plots)",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=1,
        help="이동평균 윈도우 크기 (1이면 smoothing 없음)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="그래프 창을 띄우지 않고 파일로만 저장",
    )
    args = parser.parse_args()

    log_root = args.log_root.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else (log_root / "plots")

    if not log_root.exists():
        raise FileNotFoundError(f"로그 폴더를 찾을 수 없습니다: {log_root}")

    event_files = find_event_files(log_root)
    if not event_files:
        raise FileNotFoundError(f"이벤트 파일을 찾을 수 없습니다: {log_root}")

    print(f"[INFO] 로그 루트: {log_root}")
    print(f"[INFO] 이벤트 파일 개수: {len(event_files)}")

    all_runs: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]] = {}
    for event_file in event_files:
        run_name = build_run_name(log_root, event_file)
        print(f"[LOAD] {run_name} <- {event_file.name}")
        all_runs[run_name] = load_scalars(event_file)

    plot_metrics(
        all_runs=all_runs,
        out_dir=out_dir,
        smooth_window=max(1, args.smooth_window),
        show_plot=not args.no_show,
    )

    print(f"\n[DONE] 그래프 저장 경로: {out_dir}")


if __name__ == "__main__":
    main()
