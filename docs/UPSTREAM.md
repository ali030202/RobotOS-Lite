# 开源参考与复现边界

本项目以 ROS 2 Jazzy 的官方 [minimal_action_server](https://github.com/ros2/examples/tree/jazzy/rclcpp/actions/minimal_action_server) 为 Action Server API 参考。上游仓库 `ros2/examples` 为 Apache-2.0 许可。本项目的移动任务、定时控制与状态机由本仓库重新实现，没有复制该示例源码。

故障注入思路参考 [reeceholland/ros2_fault_injection](https://github.com/reeceholland/ros2_fault_injection)（MIT 许可）：通过可控的关节状态数据中断验证控制器的响应。本项目没有移植其插件、代理或代码；仅实现两轴仿真器中的演示开关和对应测试。

检索时还看到 [Gitlnr/ros2-robot-control-dashboard](https://github.com/Gitlnr/ros2-robot-control-dashboard)，但仓库主要是文档和 ZIP，且根目录没有许可证文件；[namanharshwal/ros2-robot-dashboard](https://github.com/namanharshwal/ros2-robot-dashboard) 与网页遥测场景接近，但技术栈和本项目不同且未见根目录许可证。因此没有直接复制这些仓库的内容。

**本项目新增**：单任务两轴控制与软件急停状态机、传感器/心跳超时与锁存复位、命令看门狗、FastAPI 同源运维界面、无 ROS 依赖的控制核心单元测试和 API 测试。仅限仿真，不适用于真实设备安全控制。

