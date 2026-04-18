# R550PLUS 三全向轮机器人 ROS 包

## 概述

`turn_on_wheeltec_robot` 当前包含两条可并行使用的能力：

- 传感器数据采集：发布 `/odom`、`/imu`、`/PowerVoltage`、`/current_data`，并支持 CSV 采集
- 最小 Web 控制闭环：浏览器通过 rosbridge 连接 ROS，页面发布 `/cmd_vel_web`，机器人侧再经安全适配后输出 `/cmd_vel`

这一版 Web 功能刻意保持轻量，目标是先打通一个安全的 MVP，而不是一次性演进成完整远程运维平台。

---

## 当前接口

### 串口约定

| 设备 | 默认串口 | 波特率 | 说明 |
|------|----------|--------|------|
| 底盘控制器 | `/dev/wheeltec_controller` | 115200 | 与 `base_serial.launch` 保持一致，建议通过 udev 别名固定 |
| 电流采集板 | `/dev/ttyCH343USB1` | 115200 | 由 `current_reader.py` 读取 |

### ROS 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/odom` | `nav_msgs/Odometry` | 里程计 |
| 发布 | `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| 发布 | `/PowerVoltage` | `std_msgs/Float32` | 电池电压 |
| 发布 | `/current_data` | `std_msgs/Float32MultiArray` | 三通道电流 |
| Web 输入 | `/cmd_vel_web` | `geometry_msgs/Twist` | 浏览器控制输入 |
| Web 输入 | `/web/heartbeat` | `std_msgs/Empty` | 浏览器心跳 |
| Web 输入 | `/web/estop` | `std_msgs/Bool` | 浏览器急停锁存/解除 |
| Web 状态 | `/web/control_status` | `std_msgs/String` | Web 控制状态 |
| 最终控制 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘节点实际消费的话题 |

---

## 系统结构

```text
                         串口设备
                              │
          ┌───────────────────┴───────────────────┐
          │                                       │
          ▼                                       ▼
  ┌───────────────┐                     ┌─────────────────┐
  │wheeltec_robot │                     │ current_reader  │
  │    _node      │                     │      .py        │
  │    (C++)      │                     │    (Python)     │
  └───────┬───────┘                     └────────┬────────┘
          │                                      │
          │  /odom /imu /PowerVoltage            │  /current_data
          └───────────────────┬──────────────────┘
                              │
                     ┌────────▼────────┐
                     │ data_collector  │
                     │      .py        │
                     └─────────────────┘

浏览器 WebSocket
        │
        ▼
  rosbridge_websocket
        │
        ▼
  /cmd_vel_web + /web/heartbeat + /web/estop
        │
        ▼
  cmd_vel_web_adapter.py
        │  (限速 / 超时停车 / 急停锁存)
        ▼
      /cmd_vel
        │
        ▼
  wheeltec_robot_node
```

---

## 数据采集流程

### 前提条件

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
```

### 启动方式 1：一键采集

```bash
roslaunch turn_on_wheeltec_robot data_collector.launch
```

这个 launch 会启动：

- 底盘串口节点
- 电流采集节点
- CSV 数据采集节点

### 启动方式 2：分开启动

终端 1：

```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

终端 2：

```bash
rosrun turn_on_wheeltec_robot current_reader.py
```

终端 3：

```bash
rosrun turn_on_wheeltec_robot data_collector.py
```

### CSV 字段

```text
timestamp,x,y,z,vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,current0,current1,current2
```

### 默认输出目录

- `/home/wheeltec/R550PLUS_data_collect/log/`

---

## Web 控制 MVP

### 功能范围

- 单页浏览器控制界面，适配手机和桌面端
- 虚拟摇杆控制（前进/后退 + 转向单摇杆）
- 浏览器原生 Xbox 手柄支持（Gamepad API）
- 急停与解除急停
- 电池、电流、里程计、IMU 基础监控
- 机器人侧命令超时保护、限速和更灵敏的响应曲线

当前不包含：

- 视频传输
- 多客户端仲裁
- 导航与 Web 控制并行管理

### 新版 React 控制台

仓库现在同时包含一个新的前端源码工程：

- 源码目录：`turn_on_wheeltec_robot/webapp`
- 构建输出：`turn_on_wheeltec_robot/web/dist`

技术栈：

- React + TypeScript + Vite
- Tailwind CSS
- Zustand
- Recharts
- roslib

新的静态服务策略：

- 如果 `web/dist` 存在，`web_dashboard_server.py` 会优先服务新的 React 控制台
- 如果 `web/dist` 不存在，则自动回退到旧版 `web/index.html`

开发与迁移文档：

- [webapp/README.md](./webapp/README.md)
- [webapp/MIGRATION.md](./webapp/MIGRATION.md)

### 部署

新版控制台的部署原则很简单：

- 前端源码在 `turn_on_wheeltec_robot/webapp`
- 前端构建产物输出到 `turn_on_wheeltec_robot/web/dist`
- `web/dist` 默认作为本地构建产物使用，不建议直接提交 `node_modules` 或其它前端生成缓存
- `web_dashboard_server.py` 会优先服务 `web/dist`
- `web/dist` 不存在时自动回退到旧版 `web/index.html`

#### 方式 1：在机器人本机直接部署

适合机器人本身已经安装了 Node.js，且你希望在机器人上直接构建前端。

前提：

- 机器人已安装 Node.js 和 npm
- ROS 工作空间可以正常 `catkin_make`

部署步骤：

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash

cd src/turn_on_wheeltec_robot/webapp
npm install
npm run build

cd ~/ml_robot_ws
source devel/setup.bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

启动后访问：

- 页面：`http://<robot-ip>:8000`
- rosbridge：`ws://<robot-ip>:9090`

#### 方式 2：在开发机构建，再拷贝到机器人

适合机器人性能较弱，或你希望把构建过程放在开发机完成。

开发机构建：

```bash
cd turn_on_wheeltec_robot/webapp
npm install
npm run build
```

构建完成后会生成：

- `turn_on_wheeltec_robot/web/dist`

然后把整个 `dist` 目录同步到机器人对应 ROS 包下的 `web/` 目录中，保证机器人上的路径最终为：

```text
turn_on_wheeltec_robot/web/dist
```

之后在机器人上执行：

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

#### 部署后验证

建议按下面顺序检查：

1. 浏览器能否打开 `http://<robot-ip>:8000`
2. 页面顶部连接状态是否从 `Disconnected` 进入 `Connected`
3. `/web/control_status` 是否有状态输出
4. 急停按钮是否能让机器人进入锁存急停
5. Recorder 是否能开始录制、停止录制并下载 CSV
6. 手机端触控摇杆与 Windows 端 Gamepad 是否能分别接管控制

#### 回退到旧版页面

如果新前端部署后需要快速回退：

1. 删除或重命名 `turn_on_wheeltec_robot/web/dist`
2. 重启 `web_dashboard_server.py` 或重新执行 `roslaunch turn_on_wheeltec_robot web_control.launch`
3. 服务会自动回退到旧版 `turn_on_wheeltec_robot/web/index.html`

### 启动

```bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

默认端口：

- Web 页面：`http://<robot-ip>:8000`
- Rosbridge：`ws://<robot-ip>:9090`

### 机器人侧安全逻辑

`cmd_vel_web_adapter.py` 会把浏览器输入转换成底盘实际命令，并增加三层最小保护：

- 命令超时：默认 0.5 秒无新指令则停车
- 心跳超时：默认 1.0 秒无心跳则停车
- 速度限幅：默认限制 `linear.x`、`linear.y`、`angular.z`

`/web/estop` 为锁存式急停：

- `true`：进入急停，持续输出零速度
- `false`：解除急停

### 使用建议

- 这一版建议与导航控制分开使用，避免多个来源同时写入 `/cmd_vel`
- 浏览器控制是便利入口，不应替代物理急停
- 真机测试前先把限速参数调小

### Xbox 手柄映射

浏览器检测到 Xbox 手柄后，会在页面中显示控制器状态，并自动作为第二输入源接入现有 Web 控制链路：

- 左摇杆：前进/后退 + 转向
- `A`：立即停车
- `B`：锁存急停
- `X`：解除急停

说明：

- 当你正在拖动页面摇杆时，触摸输入优先
- 松开页面摇杆后，Xbox 手柄可直接接管
- 依然通过 `/cmd_vel_web`、`/web/heartbeat`、`/web/estop` 进入机器人侧安全适配节点

---

## 参数配置

### current_reader.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~port` | `/dev/ttyCH343USB1` | 电流采集串口 |
| `~baud` | `115200` | 波特率 |
| `~timeout` | `1.0` | 串口超时（秒） |

示例：

```bash
rosrun turn_on_wheeltec_robot current_reader.py _port:=/dev/ttyCH343USB1 _baud:=115200
```

### data_collector.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~output_dir` | `/home/wheeltec/R550PLUS_data_collect/log` | 输出目录 |
| `~rate` | `10` | 采样频率（Hz） |
| `~window_size` | `50` | 滑动窗口大小 |
| `~step_size` | `10` | 滑动步长 |

### web_control.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rosbridge_port` | `9090` | rosbridge 端口 |
| `web_port` | `8000` | 静态 Web 页面端口 |
| `publish_rate` | `40.0` | 机器人侧控制发布频率（Hz） |
| `cmd_timeout` | `0.5` | 命令超时停车阈值 |
| `heartbeat_timeout` | `1.0` | 心跳超时停车阈值 |
| `max_linear_x` | `0.55` | 最大前后速度 |
| `max_linear_y` | `0.35` | 最大横移速度 |
| `max_angular_z` | `1.60` | 最大角速度 |
| `linear_deadband` | `0.02` | 机器人侧线速度死区 |
| `angular_deadband` | `0.03` | 机器人侧角速度死区 |
| `response_exponent` | `0.70` | 响应曲线指数，越小越灵敏 |
| `min_linear_ratio` | `0.20` | 一旦脱离死区后的最小前进响应比例 |
| `min_angular_ratio` | `0.24` | 一旦脱离死区后的最小转向响应比例 |

---

## 调试命令

### 查看话题

```bash
rostopic list | grep -E "odom|imu|PowerVoltage|current|cmd_vel|web"
```

### 查看实时数据

```bash
rostopic echo /odom
rostopic echo /imu
rostopic echo /PowerVoltage
rostopic echo /current_data
rostopic echo /web/control_status
```

### 查看频率

```bash
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data
```

### 检查串口

```bash
cat /dev/wheeltec_controller
cat /dev/ttyCH343USB1
```

---

## 常见问题

### 1. 浏览器能打开，但无法连接机器人

- 确认 `web_control.launch` 已启动
- 确认 `9090` 端口可访问
- 确认页面中的 WebSocket 地址是机器人实际 IP
- 确认 `rosbridge_server` 已安装
- 如果是新前端，确认 `turn_on_wheeltec_robot/web/dist` 已构建完成

### 2. 电流数据为空

```bash
rostopic echo /current_data
ls -l /dev/ttyCH343USB1
cat /dev/ttyCH343USB1
```

### 3. 里程计或 IMU 无数据

```bash
rostopic echo /odom
cat /dev/wheeltec_controller
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

### 4. 浏览器松开摇杆后车不停车

- 检查 `/web/heartbeat` 是否正常发送
- 检查 `/web/control_status` 是否进入 `cmd_timeout` 或 `heartbeat_timeout`
- 确认页面不是多个标签页同时发送命令

### 5. 页面仍然是旧版样式

- 检查机器人上的 `turn_on_wheeltec_robot/web/dist` 是否存在
- 检查 `npm run build` 是否成功完成
- 重启 `web_control.launch`
- 确认 `web_dashboard_server.py` 没有报错并且优先指向 `web/dist`

### 6. 新页面能打开，但 Recorder 文件列表为空

- 检查 `data_collector.py` 是否已启动
- 检查 launch 中的 `data_output_dir` 是否正确
- 检查 `http://<robot-ip>:8000/api/data/list` 是否能返回 JSON
- 检查机器人目录 `/home/wheeltec/R550PLUS_data_collect/log/` 下是否真的有 CSV 文件

---

## 文件清单

```text
turn_on_wheeltec_robot/
├── src/
│   └── wheeltec_robot.cpp          # 底盘控制节点源码
├── scripts/
│   ├── current_reader.py           # 电流读取脚本
│   ├── data_collector.py           # 数据采集脚本
│   ├── cmd_vel_web_adapter.py      # Web 控制安全适配节点
│   └── web_dashboard_server.py     # 静态 Web 页面服务
├── web/
│   ├── index.html                  # 旧版 Web 控制与监控页面
│   └── dist/                       # 新版 React 控制台构建产物
├── webapp/                         # 新版前端源码工程
└── launch/
    ├── data_collector.launch       # 数据采集启动文件
    ├── web_control.launch          # Web 控制 MVP 启动文件
    └── include/base_serial.launch  # 底盘串口配置
```

---

## 作者

杨鹏 - 燕山大学本科毕业设计
