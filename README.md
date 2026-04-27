# logistics_robot
基于turtlebot3的物流机器人


# 初步需求
物流场景机器人，二次开发TurtleBot3项目

主要任务：前往地图中指定地方，视觉识别物体，机械手自动抓取，返回指定地方。

技术栈：Socket，WebSocket，Qt，DDS，ZMQ，C++14，Python，Gtest，Realsense，PCL，SLAM，Nav2，Movelt2，BehaviorTree.CPP，docker

1.仓储场景仿真环境，机械臂机器人<br>
2.运动控制模块<br>
3.路径规划模块<br>
4.任务调度模块<br>
5.视觉作业模块<br>
6.机械臂模块<br>
7.本地可视化与远程监控

# 整体架构
~~~
logistics_robot_ws/
├── src/
│   ├── robot_base/                   # 基础支撑层（全系统通用能力）
│   ├── robot_interfaces/             # 全系统标准化接口定义（消息/服务/动作）
│   ├── robot_hardware_hal/           # 硬件抽象层HAL（隔离硬件与上层）
│   ├── robot_execution/              # 执行层（硬件指令执行与反馈采集）
│   ├── robot_perception/             # 感知层（环境感知与状态感知）
│   ├── robot_control/                # 控制层（运动与执行器闭环控制）
│   ├── robot_decision/               # 决策层（任务调度与行为决策）
│   ├── robot_hmi/                    # 人机交互层（外部系统对接与可视化）
│   └── robot_bringup/                # 启动配置层（launch/参数/URDF模型）
├── docs/                             # 接口文档、调试手册、故障码说明
├── scripts/                          # 校准脚本、升级脚本、环境配置脚本
├── tests/                            # 单元测试、集成测试、仿真测试用例
└── README.md
~~~