# RobotOS-Lite

**个人项目 / Personal project** — 基于 ROS 2 Jazzy、C++、FastAPI 与 Vue 3 的两轴机器人软件仿真原型，用于学习模块通信、任务控制、远程状态监控和故障恢复测试。

项目参考开源示例进行功能复现并扩展，加入独立控制状态机、软件急停、超时检测、浏览器运维页面以及自动化测试。来源与实现边界见 [开源参考说明](docs/UPSTREAM.md)。

## 功能

- **机器人控制**：C++ 仿真节点和控制节点；ROS Topic 发布关节状态与心跳；Action 执行移动任务并提供反馈、取消和结果；Service 处理软件急停、故障复位。
- **状态与故障**：目标/速度限幅、单任务互斥、传感器 0.2 秒中断时停止运动输出、1 秒持续中断后锁存故障、250 ms 仿真命令看门狗。
- **浏览器运维**：FastAPI HTTP API、只读 WebSocket、Vue 实时状态页面，支持移动、取消、软件急停和手动复位。
- **环境适配**：支持 API 在 Conda Python 3.9 中运行，通过本机 Unix socket 连接系统 Python 3.12 的 ROS 桥接进程。
- **自动化验证**：GoogleTest、pytest 接口测试、真实 API 操作测试与直接 ROS 故障恢复测试。

## 架构

```mermaid
flowchart LR
    UI[Vue 3 运维页面] <-->|HTTP / WebSocket| API[FastAPI]
    API <-->|本机 Unix socket| Bridge[ROS Python 桥接]
    Bridge <-->|Topic / Action / Service| Controller[C++ 控制节点]
    Controller <-->|速度命令 / 关节状态| Sim[C++ 两轴仿真器]
```

## 环境

- Ubuntu 24.04、ROS 2 Jazzy、GCC 13、colcon。
- Node.js 18.19.1 或兼容版本、npm。
- Conda Python 3.9 用于 API；Ubuntu 系统 Python 3.12 用于 ROS 原生绑定。

本项目控制的是两轴位置积分仿真器，尚未接入真实机器人或硬件安全回路。

## 快速开始

以下命令在 Ubuntu 中执行。请先按 [ROS 2 Jazzy 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)安装 ROS。

```bash
git clone https://github.com/ali030202/RobotOS-Lite.git
cd RobotOS-Lite
sudo apt-get install ros-dev-tools ros-jazzy-ament-cmake-gtest python3-pytest
conda create -n robotos python=3.9 -y
conda activate robotos
python -m pip install -r backend/requirements.txt
bash scripts/build_ros.sh
cd frontend
npm ci
npm run build
cd ..
```

编译产物默认写入 `~/robotos-runtime`，可通过 `ROBOTOS_RUNTIME_DIR` 配置。使用英文编译目录可避免部分 ROS 接口生成工具对非 ASCII 路径的兼容问题。

创建仅供本机演示使用的操作口令：

```bash
mkdir -p .server-run
chmod 700 .server-run
python -c "import os,secrets; p='.server-run/operator-token'; fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600); os.write(fd,secrets.token_hex(24).encode()); os.close(fd)"
```

终端 1：

```bash
bash scripts/run_ros.sh
```

终端 2：

```bash
conda activate robotos
bash scripts/run_server.sh
```

浏览器打开 `http://localhost:8000`。只读状态无需口令；点击移动、取消等按钮前，在页面输入 `.server-run/operator-token` 中的值。口令仅保存在本次页面内存中。

`run_server.sh` 默认使用 `~/anaconda3/etc/profile.d/conda.sh`；其他 Conda 安装位置可通过 `ROBOTOS_CONDA_SH` 配置，环境名通过 `ROBOTOS_CONDA_ENV` 配置。部署与停止方式见 [运行说明](docs/SERVER.md)。

## 测试

C++ 测试包含在 `scripts/build_ros.sh` 中。API 与桥接通信测试：

```bash
conda activate robotos
PYTHONPATH=backend PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest backend/tests -q
```

真实 API 操作测试需要先启动 ROS 与 API；测试会操作仿真器并注入故障：

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTOS_RUNTIME_DIR:-$HOME/robotos-runtime}/install/setup.bash"
export ROBOTOS_OPERATOR_TOKEN="$(cat .server-run/operator-token)"
PYTHONPATH="backend:$PYTHONPATH" ROBOTOS_E2E=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/e2e -q
```

直接 ROS 故障恢复测试使用系统 Python：

```bash
/usr/bin/python3 -m pytest tests/integration -m integration -q
```

2026-11-12 在 Ubuntu 24.04 上实测通过 **14 项自动化用例**，另验证了控制节点退出后的离线检测。详细结果与经过本机路径脱敏的日志见 [验证记录](docs/VALIDATION.md)。

## 接口

| 接口 | 用途 |
|---|---|
| `GET /api/v1/status` | 机器人状态、关节位置、故障与数据年龄 |
| `WS /ws/v1/status` | 只读实时状态 |
| `POST /api/v1/tasks/move` | 发起两轴移动任务 |
| `GET /api/v1/tasks/{id}` | 查询任务进度和终态 |
| `POST /api/v1/tasks/{id}/cancel` | 取消任务 |
| `POST /api/v1/estop` | 软件急停或解除 |
| `POST /api/v1/fault/reset` | 手动复位故障 |

写操作需要 `X-Operator-Token`。运行后访问 `/docs` 查看 OpenAPI 文档。任务记录保存在内存，重启后不保留；HTTP 504 表示操作是否送达尚未确认，应先观察状态再决定后续操作。

## 范围

这是个人学习和工程实践项目。当前没有真实硬件、生产级账户系统、TLS、长期运行验证或安全认证。服务默认只监听本机 loopback；软件急停不能替代实体机器人的安全回路。

## License

[MIT](LICENSE)。开源参考项目的许可证与来源见 [UPSTREAM.md](docs/UPSTREAM.md)。
