# RoPet_ESPS3_AI_EYE

## 当前版本更新

这一章只补充当前工程相对原 README 已经新增和验证过的能力，下面原始 README 内容完整保留。

### 当前重点支持

- 主要开发和验证板型：`doit-esp32s3-eye-6824`、`doit-esp32s3-eye-8311`
- 已支持圆屏：`GC9A01 240x240`（1.28 寸）、`GC9A01 160x160`（0.71 寸）
- 当前工程已包含语音、视觉、表情、舵机动作、MCP 控制和 OTA 的完整链路

### 已新增的主要功能

- 支持 `MQTT` 和 `WebSocket` 两种协议配置下发
- 支持 OTA 激活、配置拉取和固件升级
- 支持设备端 AEC 和服务器端 AEC 切换
- 支持哼唱模式，TTS 可切换为无词哼唱，并保存到 NVS
- 支持 13 种眼球风格切换，并支持眼球锁定、扫描、转圈控制
- 支持相机拍照、图片理解、拍照后眼屏预览
- 0.71 寸预览图当前采用“保留原图比例，居中停留，左右扫视，再回中间停留”的显示方式
- 支持 4 舵机基础动作、步态动作以及两套完整舞蹈：`Smooth Criminal`、`Single Ladies`
- 已集成本地 `Helix MP3` 解码器用于舞蹈音乐播放

### 已开放的 MCP 能力

- `self.get_device_status`
- `self.audio_speaker.set_volume`
- `self.audio_speaker.toggle_humming_mode`
- `self.screen.set_brightness`
- `self.screen.set_theme`
- `self.screen.eye_style`
- `self.screen.eye_control`
- `self.camera.take_photo`
- `self.servo.stand` / `rest` / `walk` / `turn` / `turn_around`
- `self.servo.bounce` / `sway` / `tiptoe` / `shake`
- `self.servo.moonwalk` / `crusaito` / `flap` / `ascend`
- `self.servo.dance_smooth_criminal`
- `self.servo.dance_single_ladies`
- `self.servo.stop`

### 当前建议查看的文档

- `docs/build_and_config.md`：当前板型构建与关键配置说明
- `docs/otto_servo_mapping.md`：Otto 动作到本项目舵机映射参考

*   首先致谢虾哥的开源项目：https://github.com/78/xiaozhi-esp32
*   其次致谢：https://github.com/xinnan-tech/xiaozhi-esp32-server


## 项目简介
ESP32-S3作为主控，驱动两个1.28寸双目，并且支持4个触目节点，软件硬件全开源

特色：

1. 可以接入小智的服务端，也可以接入自已部署的开源小智服务端。
2. 根据聊天的情绪互动双目的表情。
3. 带4个触摸，可以和双目互动。
4. 可以扩展加摄像头支持多模态。

## 效果视频
https://www.bilibili.com/video/BV1zcV2zLEEb/

https://www.bilibili.com/video/BV1BcV2zLEaf/

## 使用说明
1. 获取代码：git clone https://github.com/SmartArduino/RoPet_ESPS3_AI_EYE.git
2. 使用vscode打开工程（需espidf版本>5.4.1）,设置目标芯片为esp32s3，命令：idf.py set-target esp32s3
3. 修改menuconfig:idf.py menuconfig
    --板子类型选择（ES8311/VB6824）
        -ES8311:将Xiaozhi Assistant->Board Type设置为doit-esp32s3-eye,并开启使用魔眼界面风格(ES8311)
        ![alt text](docs/photo/doit-eye-8311.png)
        -VB6824:将Xiaozhi Assistant->Board Type设置为doit-esp32s3-6824,并开启使用魔眼界面风格(VB6824)
        ![alt text](docs/photo/doit-eye-6824.png)
    --屏幕类型选择（1.28/0.71）
        -1.28寸:将Xiaozhi Assistant->LCD_Type设置为【GC9A01，分辨率240*240，圆屏】
        ![alt text](docs/photo/1.28.png)
        -0.71寸：将Xiaozhi Assistant->LCD_Type设置为【GC9A01，分辨率160*160，0.71寸】
        ![alt text](docs/photo/0.71.png)
4. 编译工程：idf.py build
5. 烧录代码:idf.py flash


## 硬件部分
查看工程 docs目录

## 技术支持
![alt text](docs/photo/image.png)

## 购买链接
https://item.taobao.com/item.htm?id=920076138845
