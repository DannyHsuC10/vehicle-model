"""Launch the desktop workbench or solve and export a case without a GUI."""

import argparse
from pathlib import Path

from .model import EXAMPLES, load_model, make_motion
from .results import export_results, motion_ratio
from .solver import solve


def main():
    parser = argparse.ArgumentParser(description="獨立硬點運動學解算器")
    parser.add_argument("--hardpoints", type=Path, help="硬點 CSV，省略時使用內附懸吊範例")
    parser.add_argument("--mechanism", type=Path, help="自訂機構 JSON")
    parser.add_argument("--output", type=Path, help="直接求解並將結果寫入指定資料夾")
    parser.add_argument("--duration", type=float, help="工況秒數")
    parser.add_argument("--samples", type=int, help="時間取樣數")
    parser.add_argument("--amplitude", nargs=2, metavar=("DRIVE", "MM"), action="append", default=[],
                        help="覆蓋一個驅動振幅，可重複設定")
    parser.add_argument("--profile", nargs=2, metavar=("DRIVE", "PROFILE"), action="append", default=[],
                        help="設定一項行程曲線（pulse、sine 或 ramp），可重複設定")
    parser.add_argument("--pair", nargs=2, metavar=("INPUT", "OUTPUT"), help="運動比的輸入及輸出點")
    parser.add_argument("--directions", nargs=2, metavar=("INPUT_DIR", "OUTPUT_DIR"), default=("z", "z"),
                        help="運動比的兩個量測方向，預設為 z z")
    parser.add_argument("--ratio-drive", help="運動比使用指定驅動的 Jacobian 偏導數")
    args = parser.parse_args()

    if args.mechanism and not args.hardpoints:
        parser.error("--mechanism 需要 --hardpoints。")
    if args.ratio_drive and not args.pair:
        parser.error("--ratio-drive 需要 --pair。")
    if args.output is None:
        if args.duration is not None or args.samples is not None or args.amplitude or args.profile or args.pair:
            parser.error("命令列工況選項需要 --output。")
        from .gui import launch
        launch(args.hardpoints, args.mechanism)
        return

    try:
        model = load_model(args.hardpoints or EXAMPLES / "hardpoints.csv", args.mechanism)
        motion = make_motion(model, duration=args.duration, samples=args.samples,
                             amplitudes_mm=dict(args.amplitude), profiles=dict(args.profile))
        result = solve(model, motion)
        ratio = None
        if args.pair:
            if args.ratio_drive and args.ratio_drive not in {d.name for d in model.drives if not d.locked}:
                raise ValueError("運動比的驅動必須是機構中的非鎖定驅動。")
            ratio = motion_ratio(result, *args.pair, *args.directions, drive=args.ratio_drive)
        for path in export_results(args.output, result, ratio):
            print(path)
    except (OSError, ValueError, ArithmeticError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
