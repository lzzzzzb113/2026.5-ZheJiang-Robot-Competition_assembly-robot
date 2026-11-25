# 2026.5 浙江省机器人竞赛 · 装配机器人

![MCU](https://img.shields.io/badge/MCU-STM32H723%20%7C%20MSPM0G3507-c80000?style=flat-square)
![Vision](https://img.shields.io/badge/Vision-YOLO%20%2B%20OpenCV-1769aa?style=flat-square)
![Status](https://img.shields.io/badge/Status-Competition%20archive-informational?style=flat-square)

本仓库记录 2026 年 5 月浙江省机器人竞赛装配机器人项目的软件与硬件资料，主要包括 STM32H723 主控程序、MSPM0G3507 电机与夹爪 FOC 控制、YOLO/OpenCV 视觉识别、串口通信协议、EDA 工程文件，以及用于学习和对照的参考项目。

仓库保留的是比赛期间的工程快照。各目录来自多个开发阶段，部分参数、硬件接线和版本关系需要结合实物核对；目前没有提供一键复现全部系统所需的完整标定数据。

## 系统组成

```text
USB 摄像头
    │
    ▼
PC：YOLO 形状/材质识别 + OpenCV 几何角度计算
    │  USB 串口：STX / payload / ETX，9600 baud
    ▼
STM32H723 主控：YzOS、机械臂运动、传送带、夹爪、UI、传感器
    │  位置与电机接口
    ├── 机械臂、方形/圆形/三角形/梯形物料
    ├── 内外部物料台与装配轮盘
    └── MSPM0G3507 FOC 控制板（方向电机 / 动力夹爪变体）
```

### STM32H723 主控

主控程序基于 STM32 H7 HAL、YzOS 和 LVGL 相关组件，包含以下职责：

- 机械臂 X/Y/Z/O 轴运动控制、限位检查、同步等待和抓取/放置动作序列。
- 传送带、红外传感器、外部物料台和装配轮盘状态处理。
- 通过串口接收摄像头检测结果，并进行目标跟踪、贝叶斯类型过滤和位置换算。
- LCD 界面、参数菜单、SD 卡和日志相关辅助代码。
- 方形、圆形、三角形、梯形，以及金属/塑料物料的分类抓取流程。

`R320`、`STM32H723`、`STM32H7` 是三个相近但内容不同的主控工程快照，源码和工程配置互有差异。使用前应根据实际开发板、引脚和调试器选择对应目录。

### MSPM0G3507 FOC

两个 MSPM0G3507 工程基于 TI DriverLib、CMSIS、IQmath 和 YzOS：

- `mspm0-g3507-foc-dir_motor`：包含 MT6826S 角度采样、位置环、速度环和电流环，以及串口零点/参数处理；代码已配置级联 PID 控制链路。
- `mspm0-g3507-foc-power_claw`：面向动力夹爪，当前快照主要使用电流环，串口可设置电流目标；位置环、速度环、角度更新和 `yzl_foc_init()` 在现有代码中被注释，仍属于开发中的变体。

两个工程共享 FOC PWM、PID、QMI8658 和 UART 包处理等组件。实际使用时必须核对 PWM 相序、ADC 电流方向、角度传感器安装方向、电机极对数和保护电流。

### 视觉识别与串口

视觉程序位于 `shape_grasp_project`，使用 Ultralytics YOLO 完成形状检测，再结合 OpenCV 轮廓、OBB/旋转框和几何拟合结果计算目标中心、尺寸和抓取角度。三角形和梯形还会调用单独的材质判断流程，区分金属与塑料。

主要串口协议如下：

| 字段 | 值 |
| --- | --- |
| 波特率 | `9600` |
| STX | `0x02` |
| ETX | `0x03` |
| SYN | `0x16` |
| ACK | `0x06` |

主控请求字符 `1` 至 `6` 分别对应 `ps`、`ms`、`pt`、`mt`、`si`、`yuan`。视觉端返回以 STX/ETX 包围的检测结果，默认优先选择面积较小的有效目标；具体字段和版本差异见各个 Python 入口。

### 硬件资料

根目录的 `.epro` 文件包含主控板、八通道电压采集、排插板、SCT2450 八通道板、STM32H7 连接板和装配排查等 EDA 工程。请使用与工程版本匹配的嘉立创 EDA 专业版打开。

`hardware/Dummy-Robot-main` 是用于结构和控制方案对照的第三方 Dummy-Robot 开源项目，保留其原始 README 和许可说明；它不是本队装配机器人的最终机械结构。

### 辅助实验

- `test/RobotArmANN`：四轴机械臂逆运动学数据生成与小型 ANN 实验。
- `yolo_material`：训练、数据集转换和材质检测相关脚本。
- `hello_world`：ESP-IDF Hello World 示例，仅作为工具链样例保留。

## 目录结构

```text
.
├── README.md
├── .gitignore
└── 2026COMP/
    ├── R320/                         # STM32H723 主控工程快照
    ├── STM32H723/                    # STM32H723 主控工程快照
    ├── STM32H7/                      # STM32H723 主控工程快照
    ├── mspm0-g3507-foc-dir_motor/    # 方向电机 FOC 工程
    ├── mspm0-g3507-foc-power_claw/   # 动力夹爪 FOC 工程
    ├── shape_grasp_project/          # YOLO + OpenCV 抓取视觉与串口服务
    ├── yolo_material/                # 材质模型训练与数据工具
    ├── hardware/Dummy-Robot-main/    # 第三方机械臂参考资料
    ├── test/RobotArmANN/             # 机械臂 ANN 实验
    ├── hello_world/                  # ESP-IDF 示例
    └── *.epro                        # EDA 硬件工程
```

## 关键入口

### STM32 主控

- [STM32H723 启动入口](./2026COMP/STM32H723/Core/Src/main.c)
- [R320 启动入口](./2026COMP/R320/Core/Src/main.c)
- [竞赛动作流程](./2026COMP/R320/YzOS/YzApplication/Src/RobotCupProgram.cpp)
- [机械臂与接口定义](./2026COMP/R320/YzOS/YzApplication/Inc/ArmGroup.hpp)
- [电机控制封装](./2026COMP/R320/YzOS/YzApplication/Inc/Motorclass.hpp)
- [视觉目标跟踪与贝叶斯过滤](./2026COMP/R320/YzOS/YzApplication/Src/CameraSeek.cpp)
- [物料轮盘状态机](./2026COMP/R320/YzOS/YzApplication/Src/StageStauts.cpp)

### FOC 控制

- [方向电机应用逻辑](./2026COMP/mspm0-g3507-foc-dir_motor/mspm0-g3507-foc-dir_motor/YzOS/YzApplication/Src/app.c)
- [方向电机 FOC 实现](./2026COMP/mspm0-g3507-foc-dir_motor/mspm0-g3507-foc-dir_motor/YzOS/YzApplication/Src/yzl_pwm_foc.c)
- [神经元自适应 PID](./2026COMP/mspm0-g3507-foc-dir_motor/mspm0-g3507-foc-dir_motor/YzOS/YzApplication/Src/yzl_math_pid.c)
- [动力夹爪应用逻辑](./2026COMP/mspm0-g3507-foc-power_claw/mspm0-g3507-foc-power_claw/YzOS/YzApplication/Src/app.c)

### 视觉

- [形状检测、通信和主循环](./2026COMP/shape_grasp_project/shape_grasp_project/scripts/bigbigbig.py)
- [视觉类别配置](./2026COMP/shape_grasp_project/shape_grasp_project/shape_det.yaml)
- [材质模型训练脚本](<./2026COMP/yolo_material/yolo_material/train - 副本 (2).py>)

## 构建与运行

### STM32H723

需要 ARM GNU Toolchain、`make`，以及对应版本的 STM32H7 HAL/CMSIS 文件。进入目标主控目录后执行：

```bash
cd 2026COMP/STM32H723
make -j
```

`R320` 和 `STM32H7` 也使用各自的 `Makefile`。生成的 `.elf`、`.bin` 和 `.hex` 位于 `build/`，这些构建产物不会提交到仓库。烧录前请按实际硬件修改链接脚本、调试器配置和外设引脚。

### MSPM0G3507

需要 ARM GNU Toolchain、CMake 3.22 或更高版本，以及工程中引用的 TI MSPM0 DriverLib/IQmath 文件：

```bash
cd 2026COMP/mspm0-g3507-foc-dir_motor/mspm0-g3507-foc-dir_motor
cmake -S . -B cmake-build-debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=arm-none-eabi-gcc \
  -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ \
  -DCMAKE_ASM_COMPILER=arm-none-eabi-gcc
cmake --build cmake-build-debug -j
```

动力夹爪工程将上面的目录替换为 `mspm0-g3507-foc-power_claw/mspm0-g3507-foc-power_claw`。首次上电前应先限流，并用示波器检查三相 PWM、编码器读数和电流采样，避免电机或驱动管损坏。

### 视觉端

当前没有统一的 `requirements.txt`，需要安装与显卡匹配的 PyTorch，以及以下 Python 包：

```bash
python -m pip install ultralytics opencv-python pyserial numpy
```

先将训练好的模型放到脚本配置的路径，再运行：

```bash
cd 2026COMP/shape_grasp_project/shape_grasp_project
python scripts/bigbigbig.py
```

脚本默认读取 `shape_grasp_project/runs/5184.pt` 和 `shape_grasp_project/runs/5813m.pt`，并把串口设为自动搜索、波特率 `9600`。请按注释修改 `CAMERA_ID`、模型路径和串口后运行。

## 仓库取舍与已知限制

- `*.pt` 模型权重、`runs/` 训练输出、根目录压缩包、编译目录、第三方包管理缓存和嵌套 Git 仓库未纳入提交，以避免超过 GitHub 单文件限制并减少仓库体积。
- 当前视觉入口引用的 `5184.pt`、`5813m.pt` 不在仓库中，需从本地训练产物、发布附件或其他存储恢复后才能直接运行。
- STM32 三个目录是不同阶段的工作副本，尚未确认哪一份对应最终比赛固件；仓库按原目录保留，便于比较。
- `power_claw` 当前版本的位置/速度环和部分 FOC 初始化调用被注释，不能仅根据文件名认定它已经是完整闭环位置伺服。
- 本项目未在本次整理中重新编译固件、连接硬件或复测视觉精度。比赛参数、标定表、量程、传感器方向和最终接线应以实物测试为准。
- 硬件工程和第三方参考项目可能包含各自的使用条款；本仓库根目录未指定统一开源许可证。

## 说明

本 README 依据仓库内代码、目录结构和现有说明整理。若实际比赛版本与这些快照不同，应优先以对应开发板、最终原理图和现场标定结果为准。
