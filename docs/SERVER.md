# 运行说明

本项目使用两个进程层：FastAPI 在 Conda 环境运行，ROS 桥接由系统 Python 自动启动。桥接使用权限受限的本机 Unix socket；API 退出时会关闭桥接进程。

## 配置

| 环境变量 | 默认值 | 用途 |
|---|---|---|
| `ROBOTOS_CONDA_ENV` | `robotos` | API 的 Conda 环境 |
| `ROBOTOS_CONDA_SH` | `~/anaconda3/etc/profile.d/conda.sh` | Conda 初始化脚本 |
| `ROBOTOS_RUNTIME_DIR` | `~/robotos-runtime` | ROS 编译和安装目录 |
| `ROBOTOS_PORT` | `8000` | 本机 API 端口 |
| `ROBOTOS_ROS_PYTHON` | 启动脚本设为 `/usr/bin/python3` | ROS 桥接解释器 |
| `ROBOTOS_OPERATOR_TOKEN` | 从 `.server-run/operator-token` 读取 | 写操作口令 |

`scripts/build_ros.sh` 使用干净的系统编译环境，避免 Conda Python 与 ROS 二进制包的 ABI 不匹配。`scripts/run_server.sh` 会自动激活所选 Conda 环境。

## 启动和访问

```bash
bash scripts/run_ros.sh
bash scripts/run_server.sh
```

两条启动命令应在不同终端中执行。页面地址为 `http://localhost:8000`，状态检查可用：

```bash
curl http://127.0.0.1:8000/healthz
curl http://127.0.0.1:8000/api/v1/status
```

在远程 Linux 主机上运行时，通过已有 SSH 账号创建隧道，保持终端打开：

```bash
ssh -N -L 8000:127.0.0.1:8000 user@your-server
```

然后在本机浏览器访问 `http://localhost:8000`。账户地址、口令、服务器部署目录和运行数据不包含在公开仓库中。

需要停止演示时，在对应终端按 Ctrl+C。项目未配置开机自启动；生产环境的身份认证、加密链路和硬件安全控制需要额外设计。
