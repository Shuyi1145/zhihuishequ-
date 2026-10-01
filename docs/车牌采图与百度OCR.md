# 百度车牌 OCR 配置与验证

2026-10-02 已在百度网页完成一次车牌在线调试，返回车牌号码 `冀DSX888`，`words_result` 为对象，包含车牌四角坐标与颜色 `blue`。随后在虚拟机通过 `rosservice call /recognize_plate "detect_flag: 3"` 触发实时识别，返回 `\u5180DSX888`（即 `冀DSX888`），用户提供了车牌画面截图。网页与 ROS 服务两条链路均有一次成功记录；当前两路口任务到达车牌观察位后停车，不自动调用 `/recognize_plate`。

在线调试使用的临时 `/image_raw` 单帧采图脚本已从主干移除。人物 A/B 识别后的素材采集仍只在独立分支 `codex/person-raw-capture` 中。

## 百度 API 配置

经用户明确要求，API Key 与 Secret Key 已写入仓库的 `robot_navigation/config/baidu_ocr.env`。启动文件默认把该路径传给识别节点；若进程环境中已设置 `BAIDU_OCR_API_KEY`、`BAIDU_OCR_SECRET_KEY`，则环境变量优先。虚拟机更新 Git 后执行：

```bash
cd ~/smart_ws
catkin_make
source ~/smart_ws/devel/setup.bash
roslaunch robot_navigation plate_recognition.launch
```

车牌节点读取环境变量或仓库配置文件。它从 `/image_raw` 接收图片，通过 `/recognize_plate` 服务（`detect_flag=3`）触发百度 OCR，结果图保存到 `~/smart_ws/plate_samples/`。主任务目前不会自动触发该服务。

百度接口把 `access_token` 放在请求 URL 参数中，图片以 Base64 放在表单体中。现有解析代码兼容本次实测的 `words_result` 对象格式和列表格式。虚拟机 ROS 服务的手动调用已成功一次；仍需验证任务自动触发和不同车牌、距离、角度下的稳定性。

仓库公开期间，配置文件和 Git 历史中的密钥都可能被读取。比赛结束后仅删除文件或修改仓库可见性不足以撤销已公开的密钥，应在百度控制台重置它们。
