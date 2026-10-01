# 车牌原图采集与百度 OCR 配置

当前先采图并在百度网页在线调试，任务到达第二处路口后的车牌观察位时仍停车结束，不自动调用 `/recognize_plate`。

## 保存车牌原图

Gazebo、导航和相机仍在运行，且车辆已停在面向车牌的位置时，在虚拟机新终端执行：

```bash
source ~/smart_ws/devel/setup.bash
python3 "$(rospack find robot_navigation)/scripts/save_raw_image.py" \
  --output "$HOME/smart_ws/plate_samples/plate_raw.png"
```

脚本从 `/image_raw` 等待下一帧，保存无识别框的 PNG，并打印完整路径。需要留存多张不同距离或角度的素材时，修改 `--output` 文件名，或省略该参数使用自动时间戳文件名。把保存的 PNG 从虚拟机复制到电脑，从[百度车牌识别文档](https://ai.baidu.com/ai-doc/OCR/ck3h7y191)的“在线调试”入口上传。优先上传完整相机原图；若车牌在画面中很小，可另复制一份裁剪图作对照，同时保留原图用于定位相机视角问题。

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

百度官方接口要求把 `access_token` 放在请求 URL 参数中，图片以 Base64 放在表单体中；当前接口的 `words_result` 是列表。代码已按此格式处理。现阶段没有使用用户密钥发送 API 请求，虚拟机网络与服务授权尚未验证。
