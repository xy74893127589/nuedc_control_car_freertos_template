# NUEDC 控制小车 FreeRTOS 模板

[English](README.md)

这是面向 MSPM0G3507 的模块化电赛控制小车工程。仓库只保留正式固件，
不包含硬件测试程序和测试构建入口；引脚、编码器、直线行驶及 ICM45688
参数均来自已经完成的实车测试。

## 工程特点

- 使用 FreeRTOS 静态任务，不依赖运行时动态创建任务。
- 健康监控任务统一喂 WWDT，任务失联时停止电机并触发看门狗复位。
- 电机命令超过 100 ms 未刷新时，电机模块会自动停止输出。
- ICM45688 保留原仓库已经校准完成的算法与参数。
- 左右编码器分别换算，避免强制使用同一个脉冲/厘米参数。
- 灰度、电机、编码器、IMU、遥测和视觉接口按模块分层，方便继续扩展。
- 支持 CMake、VS Code 和 J-Link 一键构建、烧录及调试。

## 目录结构

```text
app/                    FreeRTOS 任务组合、状态机和比赛流程
control/                底盘、巡线及组合控制逻辑
common/                 PID、滤波器等通用算法
config/                 FreeRTOS 配置和实车标定参数
modules/                电机、编码器、灰度、IMU、遥测和视觉模块
platform/mspm0g3507/    MSPM0G3507 驱动封装、引脚表和 SysConfig
system/                 故障管理、任务健康监控和看门狗
third_party/            TDK ICM45688 官方驱动
scripts/                构建、J-Link 烧录、调试和串口工具
docs/                   架构、引脚、标定和移植说明
```

依赖方向固定为：

```text
app -> control/模块公共接口 -> device -> platform -> TI DriverLib
```

应用层和控制层不直接使用 SysConfig 引脚宏。更换引脚时，只修改
`platform/mspm0g3507/` 下的板级绑定。

## 已验证的实车参数

所有车辆参数统一放在 `config/vehicle_calibration.h`：

| 项目 | 当前值 |
| --- | ---: |
| 左编码器 | 73.44 pulse/cm |
| 右编码器 | 73.14 pulse/cm |
| 直线基础 PWM | 左 300，右 300 |
| 编码器同步 | Kp 0.08，死区 3 pulse，最大修正 15 PWM |
| 距离减速区 | 最后 900 pulse |
| 减速最低 PWM | 220 |
| 原地转向目标 | 90° |
| 原地转向最终误差 | 0.2° |

右轮在 100 cm 场地上的实测距离为 99.8 cm。左右换算值经过实车直线
验证。巡线 PID 和轮速 PID 目前仍是初始模板值，尚未根据最终赛道和比赛
速度标定，不应视为最终比赛参数。

更完整的记录见 `docs/CALIBRATION.md`。

## 主要引脚

| 模块 | 信号 | 引脚 |
| --- | --- | --- |
| OLED | SDA / SCL | PA0 / PA1 |
| 灰度 CH1-CH7 | 从左到右 | PB20, PB24, PB25, PA24, PA25, PA26, PA27 |
| 左电机 | PWM / AIN1 / AIN2 | PA8 / PA15 / PA16 |
| 右电机 | PWM / BIN1 / BIN2 | PA9 / PA12 / PB23 |
| 左编码器 | A / B | PB15 / PB16 |
| 右编码器 | A / B | PB26 / PB27 |
| ICM45688 SPI | CS / MISO / MOSI / SCLK | PB6 / PB7 / PB8 / PB9 |
| ICM45688 | INT1 | PA13 |
| 按键 | 启停 / 模式 / 校准 | PB21 / PB1 / PA7 |
| 调试串口 | TX / RX | PA10 / PA11 |

灰度传感器为七路数字输入。完整引脚与定时器分配见 `docs/PIN_MAP.md`。

## FreeRTOS 任务与防卡死

| 任务 | 周期或来源 | 优先级 | 健康超时 |
| --- | ---: | ---: | ---: |
| health | 20 ms | 5 | 负责喂 WWDT |
| imu | DRDY，200 Hz | 4 | 100 ms |
| fast | 5 ms | 4 | 50 ms |
| control | 10 ms | 3 | 100 ms |
| telemetry | 10 ms | 2 | 500 ms |

只有健康监控任务能够喂看门狗。当任一关键任务心跳超时或系统故障被激活
时，故障管理器会停止电机，并停止喂狗，让硬件看门狗完成复位。通信和模块
接口应使用有限超时，禁止在高优先级任务中无限阻塞。

## 构建环境

需要安装：

- TI MSPM0 SDK 2.10.00.04 或兼容版本
- `arm-none-eabi-gcc`
- CMake 3.20 或更高版本
- Ninja
- SEGGER J-Link

构建正式固件：

```bash
cd nuedc_control_car_freertos_template
./scripts/build.sh
```

输出文件位于：

```text
build-fw/nuedc_control_car_freertos_template.out
build-fw/nuedc_control_car_freertos_template.hex
build-fw/nuedc_control_car_freertos_template.bin
```

检查工具链和 J-Link：

```bash
./scripts/check_env.sh
```

## VS Code 与 J-Link

在 VS Code 中打开仓库目录，然后执行 `Terminal -> Run Task`：

- `Build Main Firmware`：构建正式固件。
- `Flash Main Firmware (J-Link)`：构建并烧录正式固件。
- `Check MSPM0/J-Link Environment`：检查编译器、SDK 和 J-Link。
- `Serial Monitor 115200`：读取 Type-C 串口，默认 `/dev/ttyUSB0`。

`Ctrl+Shift+B` 默认构建正式固件。Run and Debug 中的
`Debug Main Firmware (J-Link)` 用于下载和单步调试。

命令行直接烧录：

```bash
./scripts/flash_jlink.sh
```

## 串口命令

调试串口为 UART0，115200 8N1：

```text
s 或 $START            IMU 就绪后进入 RUN
x 或 $STOP             停止小车
$DUTY,300,300,800      左右电机输出 300 PWM，持续 800 ms
$RATE,100              设置遥测周期
$RAWLINE,1             打开灰度原始数据
$RAWIMU,1              打开 IMU 原始数据
$IMUCAL,START          启动原有的偏航动态标定
$IMUCAL,CANCEL         取消标定并恢复原参数
$PAUSE / $RESUME       暂停或恢复底盘遥测
$DUMP                  输出当前遥测配置
```

## 添加新模块

1. 在 `modules/<模块名>/` 下建立独立目录。
2. 用一个公共头文件提供稳定 API，内部设备对象不对外暴露。
3. 与 MCU 无关的设备逻辑放在模块内部，板级引脚只放在 `platform/`。
4. 有可变状态的模块应由一个任务或唯一调用者管理，并向其他任务发布快照。
5. 安全相关模块必须注册健康心跳，并定义数据过期时的安全输出。
6. 通信操作必须设置有限超时，不能让控制任务永久等待。
7. 在 `CMakeLists.txt` 中显式添加源文件与头文件目录。

正式比赛流程从 `app/app_mission.c` 开始修改。更改硬件引脚前先阅读
`docs/PIN_MAP.md`，更改任务结构前先阅读 `docs/ARCHITECTURE.md`。
