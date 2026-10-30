# BeatDance：ESP32-S3 IMU 六轴数据 UDP 实时传输与可视化

## 适用硬件与接线
- ESP32-S3 Super Mini，I²C SDA=GPIO8，SCL=GPIO9。
- GY-9250 模块：实际 WHO_AM_I=0x70，按 MPU6500 六轴使用（也兼容 WHO_AM_I=0x71 的 MPU9250 六轴）。
- IMU VCC→3V3，GND→GND，SDA→GPIO8，SCL→GPIO9。
- **不发送磁力计数据**。当前硬件磁力计 0x0C 未响应。

## 最快运行步骤（Windows）
1. 电脑连接与 ESP32 相同的可相互访问的 Wi-Fi（ESP32 仅支持 2.4 GHz）。不要使用开启客户端隔离的访客网络。
2. Windows 终端运行 `ipconfig`，找到电脑当前网络适配器的 **IPv4 地址**，例如 `192.168.1.100`。不要使用路由器默认网关地址。
3. 打开 `BeatDance_IMU_UDP/BeatDance_IMU_UDP.ino`，修改 `WIFI_SSID`、`WIFI_PASSWORD`、`PC_IP`。`PC_PORT` 默认为 5005，采样/发送目标频率 50 Hz。
4. Arduino IDE 选择 ESP32-S3 开发板及串口，上传 `.ino`；串口监视器波特率 115200。无需安装第三方 Arduino 库。
5. 配置 Python 环境（需先安装 [uv](https://docs.astral.sh/uv/)）：在项目根目录运行 `uv sync`。uv 会按 `.python-version` 自动准备 Python 3.11、创建 `.venv`，并严格按照 `uv.lock` 安装 `matplotlib` 等依赖；之后用 `uv run ...` 即可运行，无需手动激活虚拟环境。
6. 运行程序：`uv run imu_udp_plot.py`
7. 如果需要记录训练数据：`uv run imu_udp_plot.py --port 5005 --csv recording.csv`。
8. 如果控制台显示 ESP32 已连接，但 Python 一直 WAITING：在 Windows Defender 防火墙中允许 Python 接收本地网络上的 UDP 5005，检查 `PC_IP` 和两台设备之间的网络互通。

> [!IMPORTANT]
> 当前Wi-Fi地址**硬编码**为:
> SSID:`BeatDance-SCUT`
> PASSWORD:`test1234`
> 请使用2.4GHz热点为下位机创造通讯环境。

## UDP 传输接口 v1
- **传输**：UDP over IPv4，单向 ESP32 → Python，局域网明文，不含认证和加密。
- **目的端**：在 ESP32 的 `PC_IP`、`PC_PORT=5005` 中指定。PC 接收绑定 `0.0.0.0:5005`，即所有本地 IPv4 网卡接口。`0.0.0.0` 不能作为 ESP32 目的地址。
- **消息边界**：每一个 UDP 数据报即一条 JSON 消息，UTF-8，不需换行符，不应假定能可靠送达或严格按序。
- **目标发送率**：50 Hz（20 ms/帧），Wi-Fi 拥塞、I²C 错误等会造成抖动或丢包。
- **内容**：当前仅六轴（加速度与角速度），附带 IMU 温度；不含地磁、姿态角或四元数。

示例：

```json
{"v":1,"device":"esp32s3-imu-01","seq":132,"t_ms":5489,"imu_id":"0x70","acc_g":[0.012,-0.008,1.001],"gyro_dps":[1.25,-0.50,0.15],"temp_c":28.5}
```

| 字段 | 类型 | 含义 | 单位/备注 |
| --- | --- | --- | --- |
| `v` | int | 数据协议版本 | 当前固定 1 |
| `device` | string | 设备标识 | 当前 `esp32s3-imu-01` |
| `seq` | uint32 | 每次成功读取 IMU 的样本序号 | 从 0 递增；断网时亦递增；用于检测序号缺口；重启后重置 |
| `t_ms` | uint32 | ESP32 毫秒时钟 | `millis()`，自开机以来，不是 Unix 时间戳；约 49.7 天回绕 |
| `imu_id` | string | `WHO_AM_I` | `0x70` 为 MPU6500，`0x71` 为 MPU9250 |
| `acc_g` | float[3] | 加速度 X、Y、Z | g，1g≈9.80665 m/s²；静止时模长≈1g，未消除重力 |
| `gyro_dps` | float[3] | 角速度 X、Y、Z | °/s，未做静态零偏校准 |
| `temp_c` | float | IMU 芯片温度估算 | °C，并非室温 |

### 量程、精度和坐标
- 加速度量程 ±4g；原始数据除以 8192 转换为 g。
- 角速度量程 ±500°/s；原始数据除以 65.5 转换为 °/s。
- 配置约 20 Hz 数字低通滤波；寄存器目标输出频率 50 Hz。
- X/Y/Z 轴是 **IMU 芯片自身的坐标系**，不是电脑屏幕/房间的世界坐标系；方向要参考实际模块丝印/安装方式。
- 示例没有进行陀螺仪零偏、加速度偏移/尺度校准，也没有计算姿态角。

### 接口行为
- 该接口 **无握手、无应答、无重传**。Python 收到消息就更新实时曲线。
- `seq` 缺口表示接收端观察到遗漏的样本序号，可能由 UDP 丢包、读取失败或设备重连造成，并不必然等于纯网络丢包。
- 两设备应放在可信的本地网络；UDP 消息无鉴权，不建议直接在公网暴露接收端口。
- Python 可通过 `--csv` 保存每个收到的样本，增加电脑接收时间 `pc_time` 便于离线分析。
- 同一端口有多个 ESP32 时本示例会将数据混在同一张图里；多设备应分别配置端口或扩展为按 `device` 分类。

## 故障排查
- `IMU initialization failed`：检查 I²C 连接、电源、地址及 NCS。
- `Wi-Fi: disconnected`：检查 Wi-Fi SSID/密码和 2.4 GHz 网络支持。
- Python 的 `WAITING FOR UDP`：核对电脑 IPv4、5005/UDP 防火墙、网络 AP 客户端隔离。
- 已有数据但数值不变：检查 IMU 是否实际运动；串口查看 `read errors`；检查供电和 I²C。
- 曲线不平滑或丢序号：检查 Wi-Fi 信号及设备负载，必要时降低发送频率。
