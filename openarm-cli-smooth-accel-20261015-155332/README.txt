OpenArm 双臂命令行测试程序 — 平滑加速版

这是自定义测试程序，不是 OpenArm 官方发布的软件包。
主入口：openarm_cli.py。接收者请从本文件开始。

1. 环境与安装
适用于 macOS 交互式终端；原环境 Python 3.9.6。
使用 termios/fcntl，不支持直接在 Windows 原生终端运行；Linux 未做实机验证。
硬件：MKS CANable V2.0，gs_usb 固件，Classic CAN 1 Mbps。
console 要求每条参与手臂有 ID 1–8 的电机，且均为位置＋速度模式 2。
程序不会自动切换模式或调整内部 PID。

在终端 cd 到解压目录，再逐条执行：
  python3 -m venv .venv-can-diagnostics
  .venv-can-diagnostics/bin/python -m pip install -r requirements.txt
安装依赖需要联网。压缩包不包含 Python 或虚拟环境。

2. 核对设备绑定
已有实验室配置时，保留本机的 openarm_cli_config.json。首次从 GitHub 下载时，复制下方模板并填写左右臂各自的 CANable 序列号。
同一套硬件更换电脑 USB 插口无需更改已核对的序列号。
只读 USB 枚举（不使能电机）：
  .venv-can-diagnostics/bin/python canable_inspect.py
只读电机参数查询（发送查询帧，不发送运动/使能命令）：
  .venv-can-diagnostics/bin/python canable_query.py --query --all-adapters
可只连接一侧 USB 确认其序列号，再分别填入配置的 left/right.serial。
不要仅凭 USB 枚举顺序判断左右。
openarm_cli_config.template.json 是空白配置模板；使用时另存为 openarm_cli_config.json。
空白模板没有已记录的姿态和已标定的机械限位，需要在实验室核对并标定。

3. 先预览
  .venv-can-diagnostics/bin/python openarm_cli.py --arm both console
这条命令不连接 USB、不使能电机。

默认最高参考速度：J1–J7 为 5°/s，J8 为 10°/s。
A/D 目标轨迹加速度：J1–J7 为 2.5°/s²，J8 为 5°/s²，约 2 秒达到上限。
松键超时/空格/S/切换关节停止推进目标；反向从零重新加速。
姿态回放使用平滑曲线。保留 3° 跟踪误差保护及速度、通信、周期保护。
参考轨迹约束不等于实测电机速度/加速度保证。

4. 需要实际运动时由操作者手动执行
先核对接线、左右绑定、运动空间，支撑机械臂并保持急停可用：
  .venv-can-diagnostics/bin/python openarm_cli.py --arm both console --speed 5 --hand-speed 10 --execute
只接一侧时将 both 换成 left 或 right。
L/R 选臂，1–8 选关节，A/D 正反向，+/- 调速，空格/S 保持，Q/Esc 退出。
全部参与关节持续位置保持；退出/故障会失能，需要机械支撑。
该程序没有重力补偿和碰撞规划；位置保持不是机械制动器。
不要同时运行其他电机控制程序或 CAN 查询程序。
完整姿态记录、回放与限制见 OPENARM_CLI_GUIDE.txt。

5. 离线验证（模拟 USB，不接硬件）
  .venv-can-diagnostics/bin/python -m unittest -q test_openarm_console_offline test_openarm_cli_offline test_canable_query_offline test_openarm_motor_test_offline test_openarm_keyboard_offline

6. 文件说明
openarm_cli.py / openarm_console.py：主程序与持续终端控制台。
openarm_motor_test.py：主程序依赖的通信与电机协议模块，必须保留。
openarm_keyboard.py：旧单电机程序的兼容依赖；主控制台不会打开网页。
请使用 openarm_cli.py，不要把旧 openarm_motor_test.py 当作本次控制台入口。
canable_inspect.py / canable_query.py：只读设备与电机诊断。
test_*_offline.py：离线测试。
requirements.txt：依赖版本。
SHA256SUMS.txt：包内文件校验值。

本包不包含历史诊断日志、虚拟环境、Git 数据或其他实验程序。
此次打包和离线验证没有使能或移动真实电机。
