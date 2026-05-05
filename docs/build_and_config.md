# doit-esp32s3-eye-6824 构建与配置指南

## Board Type

当前板级：**doit-esp32s3-eye-6824**

Board type 由 sdkconfig 中的 `CONFIG_BOARD_TYPE_DOIT_ESP32S3_EYE_6824` 决定，对应源码目录 `main/boards/doit-esp32s3-eye-6824/`。

## 关键配置项

### sdkconfig 中与当前板相关的重要选项

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `CONFIG_BOARD_TYPE_DOIT_ESP32S3_EYE_6824` | y | Board type |
| `CONFIG_LCD_GC9A01_160X160` | y | 显示屏分辨率 160×160 |
| `CONFIG_USE_EYE_STYLE_VB6824` | y | 魔眼渲染模式（双 GC9A01 SPI 屏） |
| `CONFIG_USE_AUDIO_CODEC_ENCODE_OPUS` | y | VB6824 硬编 Opus |
| `CONFIG_USE_AUDIO_CODEC_DECODE_OPUS` | y | VB6824 硬解 Opus |
| `CONFIG_USE_DEVICE_AEC` | y | 设备端 AEC |
| `CONFIG_IOT_PROTOCOL_MCP` | y | MCP 协议支持 |

### 修改配置

```bash
idf.py menuconfig
```

图形界面中浏览和修改所有 Kconfig 选项。

## 构建流程

### 前提条件

1. ESP-IDF 5.5 已安装：`/home/yuhuan/esp/esp-idf/`
2. Python 3.13 环境已配置
3. xtensa-esp32s3-elf 工具链已安装

### 构建命令

```bash
source /home/yuhuan/esp/esp-idf/export.sh
idf.py build
```

### 烧录

```bash
idf.py -p PORT flash
```

或直接 esptool（无需 idf.py）：

```bash
python -m esptool --chip esp32s3 -b 460800 --before default_reset --after hard_reset \
  write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0xd000 build/ota_data_initial.bin \
  0x10000 build/srmodels/srmodels.bin \
  0x100000 build/xiaozhi.bin
```

### 分区表

Flash 16MB，关键分区：

| 分区 | 偏移 | 大小 | 内容 |
|------|------|------|------|
| bootloader | 0x0 | 32KB | 2nd stage bootloader |
| partition_table | 0x8000 | 4KB | 分区表 |
| ota_data | 0xd000 | 8KB | OTA 状态 |
| srmodels | 0x10000 | 960KB | 语音模型 |
| app | 0x100000 | 6MB | 固件 |

## 新增的本地组件

本项目在 `components/` 目录下有一个本地组件：

| 组件 | 路径 | 说明 |
|------|------|------|
| helix | `components/helix/` | Helix MP3 解码器，用于舞蹈音乐播放 |

不受 idf_component.yml 管理，随项目源码一同编译。

## 新增的服务（MCP 工具）

### 设备控制（mcp_server.cc）
- `self.get_device_status` — 设备状态
- `self.audio_speaker.set_volume` — 音量
- `self.screen.set_brightness` — 屏幕亮度
- `self.screen.set_theme` — 明暗主题
- `self.screen.eye_style` — 眼球风格切换（default/blood/cospa/spikes/ribbon/black_star/straw）
- `self.camera.take_photo` — 拍照+AI 分析+3 秒眼球显示

### 舵机动作（servo_controller.cc）
- `self.servo.test` / `.move` — 单舵机
- `self.servo.stand` / `.rest` — 站立/稍息
- `self.servo.walk` / `.turn` / `.turn_around` — 行走/转向
- `self.servo.bounce` / `.sway` / `.tiptoe` / `.shake` — 脚部/腿部动作
- `self.servo.moonwalk` / `.crusaito` / `.flap` / `.ascend` — 特殊步态
- `self.servo.dance_smooth_criminal` — Smooth Criminal 完整舞蹈（~3 分钟，BPM 118）
- `self.servo.dance_single_ladies` — Single Ladies 完整舞蹈（~3 分钟，BPM 97）
- `self.servo.stop` — 停止当前动作

### 眼球素材

眼球数据位于 `main/eye_data/160_160/`，通过 `tablegen.py`（Adafruit Uncanny Eyes 工具）从 PNG 生成。新增风格：准备 sclera.png + iris.png（250×250），运行 tablegen.py 生成头文件，放入 `eye_data/160_160/`，在 `application.h` include 并在 `eye_style()` 注册。
