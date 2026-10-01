# 百度车牌 OCR 配置与验证

2026-10-02 已在百度网页完成一次车牌在线调试，返回车牌号码 `冀DSX888`，`words_result` 为对象，包含车牌四角坐标与颜色 `blue`。这证明上传的图片可被在线接口识别；ROS 节点在虚拟机中的实时调用仍待验证。当前两路口任务到达车牌观察位后停车，不自动调用 `/recognize_plate`。

在线调试使用的临时 `/image_raw` 单帧采图脚本已从主干移除。人物 A/B 识别后的素材采集仍只在独立分支 `codex/person-raw-capture` 中。

## 百度 API 配置

API Key 与 Secret Key 已保存到这台 Windows 电脑的私有文件 `C:\Users\DarkFlame\.config\zhihuishequ\baidu_ocr.env`，没有写入 Git。仓库中的 `robot_navigation/config/baidu_ocr.env.example` 仅包含占位符。需要在虚拟机启动 OCR 节点时，先把私有文件复制到 `~/.config/zhihuishequ/baidu_ocr.env`，并执行：

```bash
cd ~/smart_ws
catkin_make
source ~/smart_ws/devel/setup.bash
chmod 600 ~/.config/zhihuishequ/baidu_ocr.env
set -a
source ~/.config/zhihuishequ/baidu_ocr.env
set +a
roslaunch robot_navigation plate_recognition.launch
```

车牌节点读取 `BAIDU_OCR_API_KEY` 与 `BAIDU_OCR_SECRET_KEY` 环境变量。它从 `/image_raw` 接收图片，未来通过 `/recognize_plate` 服务（`detect_flag=3`）触发百度 OCR，结果图保存到 `~/smart_ws/plate_samples/`。主任务目前不会自动触发该服务。

百度接口把 `access_token` 放在请求 URL 参数中，图片以 Base64 放在表单体中。现有解析代码兼容本次实测的 `words_result` 对象格式和列表格式。网页在线调试已成功，虚拟机上的 ROS 节点、网络与服务授权尚未验证。
