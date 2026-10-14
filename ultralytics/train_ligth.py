import torch
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

print("="*60)
print(f"Python 路径: {sys.executable}")
print(f"Python 版本: {sys.version}")
print(f"PyTorch 版本: {torch.__version__}")
print(f"CUDA 可用: {torch.cuda.is_available()}")
print(f"CUDA 版本: {torch.version.cuda}")
print(f"cuDNN 版本: {torch.backends.cudnn.version()}")

try:
    print(f"PyTorch 路径: {torch.__file__}")
    print(f"CUDA 安装路径: {torch.cuda.get_installation_path()}")
except AttributeError:
    print("无法获取完整路径信息")

print("="*60)
from ultralytics import YOLO

# Load a COCO-pretrained YOLO26n model
model = YOLO(str(ROOT / "yolo26n.pt"))

# Train the model on the COCO8 example dataset for 100 epochs
results = model.train(
    data=str(ROOT / "mydata.yaml"), epochs=100, imgsz=640,
    project=str(ROOT / "runs" / "detect"), name="train", exist_ok=False,
)
