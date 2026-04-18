# WHEELTEC Robot Console Frontend

这是新的前端工程，面向 ROS 全向轮小车网页控制与监控场景，保留现有 rosbridge、录制和 CSV 下载链路，同时把旧 `web/index.html` 拆成可维护的 React + TypeScript 架构。

## 技术栈

- React 19
- TypeScript
- Vite
- Tailwind CSS
- shadcn/ui 风格基础组件
- Zustand
- Recharts
- roslib
- Browser Gamepad API

## 目录结构

```text
src/
  app/
  pages/
  widgets/
  features/
  entities/
  shared/
```

## 本地开发

```bash
cd turn_on_wheeltec_robot/webapp
npm install
npm run dev
```

默认开发地址：

- 前端：`http://localhost:5173`
- 机器人 rosbridge：`ws://<robot-ip>:9090`

说明：

- 开发模式下，前端默认仍然连接 `ws://<当前主机>:9090`
- 如果你在开发机打开页面、但 rosbridge 运行在机器人上，请确保页面访问地址和 ROS 主机地址一致，或按需要调整连接地址

推荐在本地开发时配置：

```bash
VITE_ROSBRIDGE_URL=ws://<robot-ip>:9090
VITE_API_BASE=http://<robot-ip>:8000
```

这样 Recorder 文件列表和 CSV 下载会直接指向机器人 Web 服务，而不是 Vite 开发服务器。

## 部署

### 部署目标

构建后的前端不会单独起一个 Node 服务，而是交给 ROS 包内的 `web_dashboard_server.py` 直接托管。

最终部署结构：

```text
turn_on_wheeltec_robot/
  web/
    dist/
      index.html
      assets/
```

### 方式 1：本机构建并运行

如果你就在机器人本机操作：

```bash
cd turn_on_wheeltec_robot/webapp
npm install
npm run build
```

然后回到 ROS 工作空间：

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

### 方式 2：开发机构建，拷贝到机器人

如果你在 Windows/Linux 开发机构建：

```bash
cd turn_on_wheeltec_robot/webapp
npm install
npm run build
```

构建成功后，把 `turn_on_wheeltec_robot/web/dist` 整个目录同步到机器人相同 ROS 包路径下。

机器人端只需要：

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

### 启动后访问

- 控制台页面：`http://<robot-ip>:8000`
- rosbridge：`ws://<robot-ip>:9090`

### 如何确认部署的是新版前端

满足下面任一项即可认为新版已经生效：

- 页面顶部显示新的 `WHEELTEC Robot Console`
- 页面中存在 `Dashboard / Control / Recorder` 三个导航入口
- 机器人包目录中存在 `turn_on_wheeltec_robot/web/dist/index.html`

## 构建到 ROS 包

```bash
cd turn_on_wheeltec_robot/webapp
npm install
npm run build
```

构建产物会输出到：

- `turn_on_wheeltec_robot/web/dist`

`web_dashboard_server.py` 会优先服务 `web/dist`；如果未构建，则回退到旧版 `web/index.html`。

## 回退

如果需要暂时回退到旧版页面：

```bash
mv turn_on_wheeltec_robot/web/dist turn_on_wheeltec_robot/web/dist.bak
```

然后重启：

```bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

服务会自动回退到旧版 `web/index.html`。

## 部署后排查

### 页面打不开

- 检查 `web_control.launch` 是否启动成功
- 检查 `8000` 端口是否监听
- 检查 `web_dashboard_server.py` 是否报错

### 页面打开但还是旧版

- 检查 `web/dist` 是否存在
- 检查 `npm run build` 是否成功
- 检查构建文件是否真的同步到了机器人

### 页面能打开但 ROS 连不上

- 检查 `9090` 端口
- 检查 `rosbridge_websocket` 是否成功启动
- 检查浏览器中配置的 WebSocket 地址是否为机器人 IP

## 保留的 ROS / HTTP 合约

- 发布 `/cmd_vel_web`
- 发布 `/web/heartbeat`
- 发布 `/web/estop`
- 发布 `/web/data_collect/command`
- 订阅 `/odom`
- 订阅 `/imu`
- 订阅 `/PowerVoltage`
- 订阅 `/current_data`
- 订阅 `/web/control_status`
- 订阅 `/web/data_collect/status`
- 请求 `/api/data/list`
- 下载 `/api/data/download/<filename>`
