from __future__ import annotations

import argparse
import platform
import sys
from pathlib import Path

import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "yolo26n.pt"
DEFAULT_DATA = ROOT / "mydata.yaml"
DEFAULT_PROJECT = ROOT / "runs" / "detect"


def str2bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.strip().lower()
    if lowered in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def resolve_path(path_str: str, default_root: Path = ROOT) -> Path:
    candidate = Path(path_str).expanduser()
    if candidate.is_absolute():
        return candidate
    return (default_root / candidate).resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 自定义数据训练入口")

    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="模型权重或模型配置路径")
    parser.add_argument("--data", default=str(DEFAULT_DATA), help="数据集 YAML 路径")
    parser.add_argument("--epochs", type=int, default=150, help="训练轮数")
    parser.add_argument("--imgsz", type=int, default=640, help="训练分辨率")
    parser.add_argument("--batch", type=int, default=8, help="批大小，-1 表示自动批大小")
    parser.add_argument("--device", default=0, help="训练设备，例如 cpu、0、0,1；默认自动选择")
    parser.add_argument("--workers", type=int, default=8, help="DataLoader worker 数")
    parser.add_argument("--patience", type=int, default=50, help="早停 patience")
    parser.add_argument("--cache", type=str2bool, default=False, help="是否缓存图像")
    parser.add_argument("--amp", type=str2bool, default=True, help="是否开启 AMP 混合精度")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    parser.add_argument("--deterministic", type=str2bool, default=True, help="是否使用确定性训练")
    parser.add_argument("--project", default=str(DEFAULT_PROJECT), help="训练输出根目录，相对路径以当前脚本目录为基准")
    parser.add_argument("--name", default="train", help="本次训练输出目录名")
    parser.add_argument("--exist-ok", dest="exist_ok", type=str2bool, default=False, help="输出目录已存在时是否复用")
    parser.add_argument("--resume", type=str2bool, default=False, help="是否从上次中断训练恢复")
    parser.add_argument("--pretrained", type=str2bool, default=True, help="是否使用预训练权重")
    parser.add_argument("--single-cls", dest="single_cls", type=str2bool, default=False, help="是否将所有类别视为单类")
    parser.add_argument("--rect", type=str2bool, default=False, help="是否启用矩形训练")
    parser.add_argument("--cos-lr", dest="cos_lr", type=str2bool, default=False, help="是否启用余弦学习率")
    parser.add_argument("--close-mosaic", type=int, default=10, help="最后多少个 epoch 关闭 mosaic")
    parser.add_argument("--freeze", type=int, default=None, help="冻结前多少层，默认不冻结")
    parser.add_argument("--save", type=str2bool, default=True, help="是否保存权重和结果")
    parser.add_argument("--save-period", type=int, default=-1, help="每隔多少 epoch 保存一次，-1 表示仅保存最后和最佳")
    parser.add_argument("--plots", type=str2bool, default=True, help="是否保存训练曲线和可视化")
    parser.add_argument("--verbose", type=str2bool, default=True, help="是否打印详细日志")
    parser.add_argument("--val", type=str2bool, default=True, help="训练期间是否做验证")

    parser.add_argument("--optimizer", default="auto", choices=["auto", "SGD", "Adam", "AdamW", "NAdam", "RAdam", "RMSProp"], help="优化器")
    parser.add_argument("--lr0", type=float, default=0.01, help="初始学习率")
    parser.add_argument("--lrf", type=float, default=0.01, help="最终学习率系数")
    parser.add_argument("--momentum", type=float, default=0.937, help="动量")
    parser.add_argument("--weight-decay", dest="weight_decay", type=float, default=0.0005, help="权重衰减")
    parser.add_argument("--warmup-epochs", dest="warmup_epochs", type=float, default=3.0, help="warmup epoch 数")
    parser.add_argument("--warmup-momentum", dest="warmup_momentum", type=float, default=0.8, help="warmup 初始动量")
    parser.add_argument("--warmup-bias-lr", dest="warmup_bias_lr", type=float, default=0.1, help="warmup bias 学习率")
    parser.add_argument("--box", type=float, default=7.5, help="box loss 权重")
    parser.add_argument("--cls", type=float, default=0.5, help="cls loss 权重")
    parser.add_argument("--dfl", type=float, default=1.5, help="DFL loss 权重")

    parser.add_argument("--hsv-h", dest="hsv_h", type=float, default=0.015, help="色调增强")
    parser.add_argument("--hsv-s", dest="hsv_s", type=float, default=0.7, help="饱和度增强")
    parser.add_argument("--hsv-v", dest="hsv_v", type=float, default=0.4, help="亮度增强")
    parser.add_argument("--degrees", type=float, default=0.0, help="随机旋转角度")
    parser.add_argument("--translate", type=float, default=0.1, help="随机平移比例")
    parser.add_argument("--scale", type=float, default=0.5, help="随机缩放比例")
    parser.add_argument("--shear", type=float, default=0.0, help="随机错切角度")
    parser.add_argument("--perspective", type=float, default=0.0, help="随机透视变换")
    parser.add_argument("--flipud", type=float, default=0.0, help="上下翻转概率")
    parser.add_argument("--fliplr", type=float, default=0.5, help="左右翻转概率")
    parser.add_argument("--mosaic", type=float, default=1.0, help="mosaic 概率")
    parser.add_argument("--mixup", type=float, default=0.0, help="mixup 概率")
    parser.add_argument("--copy-paste", dest="copy_paste", type=float, default=0.0, help="copy-paste 概率")
    parser.add_argument("--erasing", type=float, default=0.4, help="随机擦除概率")

    return parser.parse_args()


def collect_environment_info(requested_device: str | None) -> dict[str, object]:
    cuda_available = torch.cuda.is_available()
    device_count = torch.cuda.device_count() if cuda_available else 0
    gpu_names = [torch.cuda.get_device_name(index) for index in range(device_count)] if cuda_available else []
    selected_device = requested_device or ("0" if cuda_available else "cpu")

    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "torch": torch.__version__,
        "cuda_built": torch.version.cuda,
        "cuda_available": cuda_available,
        "device_count": device_count,
        "gpu_names": gpu_names,
        "requested_device": requested_device or "auto",
        "resolved_device": selected_device,
        "will_use_gpu": cuda_available and selected_device != "cpu",
    }


def print_environment_info(env: dict[str, object]) -> None:
    print("=" * 72)
    print("Runtime Environment")
    print(f"Python           : {env['python']}")
    print(f"Platform         : {env['platform']}")
    print(f"PyTorch          : {env['torch']}")
    print(f"CUDA Built       : {env['cuda_built']}")
    print(f"CUDA Available   : {env['cuda_available']}")
    print(f"GPU Count        : {env['device_count']}")

    gpu_names: list[str] = env["gpu_names"]  # type: ignore[assignment]
    if gpu_names:
        for index, name in enumerate(gpu_names):
            print(f"GPU[{index}]         : {name}")

    print(f"Requested Device : {env['requested_device']}")
    print(f"Resolved Device  : {env['resolved_device']}")
    print(f"Use GPU Training : {env['will_use_gpu']}")
    print("=" * 72)


def build_train_kwargs(args: argparse.Namespace, resolved_device: str) -> dict[str, object]:
    train_kwargs: dict[str, object] = {
        "data": str(resolve_path(args.data)),
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "device": resolved_device,
        "workers": args.workers,
        "patience": args.patience,
        "cache": args.cache,
        "amp": args.amp,
        "seed": args.seed,
        "deterministic": args.deterministic,
        "project": str(resolve_path(args.project)),
        "name": args.name,
        "exist_ok": args.exist_ok,
        "resume": args.resume,
        "pretrained": args.pretrained,
        "single_cls": args.single_cls,
        "rect": args.rect,
        "cos_lr": args.cos_lr,
        "close_mosaic": args.close_mosaic,
        "save": args.save,
        "save_period": args.save_period,
        "plots": args.plots,
        "verbose": args.verbose,
        "val": args.val,
        "optimizer": args.optimizer,
        "lr0": args.lr0,
        "lrf": args.lrf,
        "momentum": args.momentum,
        "weight_decay": args.weight_decay,
        "warmup_epochs": args.warmup_epochs,
        "warmup_momentum": args.warmup_momentum,
        "warmup_bias_lr": args.warmup_bias_lr,
        "box": args.box,
        "cls": args.cls,
        "dfl": args.dfl,
        "hsv_h": args.hsv_h,
        "hsv_s": args.hsv_s,
        "hsv_v": args.hsv_v,
        "degrees": args.degrees,
        "translate": args.translate,
        "scale": args.scale,
        "shear": args.shear,
        "perspective": args.perspective,
        "flipud": args.flipud,
        "fliplr": args.fliplr,
        "mosaic": args.mosaic,
        "mixup": args.mixup,
        "copy_paste": args.copy_paste,
        "erasing": args.erasing,
    }
    if args.freeze is not None:
        train_kwargs["freeze"] = args.freeze
    return train_kwargs


def print_train_kwargs(train_kwargs: dict[str, object]) -> None:
    print("Train Arguments")
    for key in sorted(train_kwargs):
        print(f"{key:16}: {train_kwargs[key]}")
    print("=" * 72)


def main() -> None:
    args = parse_args()
    model_path = resolve_path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model path not found: {model_path}")

    env = collect_environment_info(args.device)
    print_environment_info(env)

    train_kwargs = build_train_kwargs(args, str(env["resolved_device"]))
    print_train_kwargs(train_kwargs)

    model = YOLO(str(model_path))
    model.train(**train_kwargs)


if __name__ == "__main__":
    main()
