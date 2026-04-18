# Migration Guide

## 目标

把旧版单文件 `turn_on_wheeltec_robot/web/index.html` 渐进迁移到新的 `turn_on_wheeltec_robot/webapp`，同时保持 ROS 端 launch、topic 和 CSV API 不变。

## 映射关系

### 旧页面逻辑 -> 新结构

- WebSocket rosbridge 直连
  - 旧：`web/index.html`
  - 新：`src/shared/lib/ros/RosClient.ts`

- 话题订阅与 UI 更新
  - 旧：`web/index.html`
  - 新：`src/features/ros-connect/lib/register-ros-topics.ts`
  - 新：`src/shared/lib/ros/adapters.ts`
  - 新：`src/entities/robot/model/robot-store.ts`

- 虚拟摇杆 / 键盘 / 速度倍率
  - 旧：`web/index.html`
  - 新：`src/features/manual-control/**/*`

- Gamepad 轮询与映射
  - 旧：`web/index.html`
  - 新：`src/features/gamepad/**/*`

- 录制命令与文件列表
  - 旧：`web/index.html`
  - 新：`src/features/recorder/**/*`

- mini chart
  - 旧：`canvas` 绘制
  - 新：`Recharts` in `src/widgets/telemetry-grid/TelemetryGrid.tsx`

### 旧后端静态服务 -> 新方式

- 旧：`web_dashboard_server.py` 固定服务 `web/`
- 新：优先服务 `web/dist/`，未构建时回退到 `web/`

## 迁移步骤

1. 在开发机进入 `turn_on_wheeltec_robot/webapp`
2. 执行 `npm install`
3. 执行 `npm run build`
4. 确认生成 `turn_on_wheeltec_robot/web/dist`
5. 如果前端不是在机器人本机构建，把 `web/dist` 同步到机器人对应 ROS 包目录
6. 继续使用原有命令：
   - `roslaunch turn_on_wheeltec_robot web_control.launch`
7. 浏览器打开 `http://<robot-ip>:8000`

注意：

- `webapp/node_modules`、`*.tsbuildinfo`、以及配置文件的编译副产物不应提交到版本库
- `web/dist` 建议作为部署产物管理，而不是长期手工维护的源码目录

## 上线检查清单

建议在真机上线前逐项确认：

1. `turn_on_wheeltec_robot/web/dist/index.html` 已存在
2. `roslaunch turn_on_wheeltec_robot web_control.launch` 启动正常
3. `http://<robot-ip>:8000` 能打开新控制台
4. `ws://<robot-ip>:9090` 可连接
5. `http://<robot-ip>:8000/api/data/list` 能返回文件列表 JSON
6. 急停、停止、录制开始、录制停止都能正确响应
7. 手机端触控和桌面端 Gamepad 至少各验证一次

## 推荐的后续迁移顺序

1. 先验证连接、急停、录制和 CSV 下载
2. 再验证手机端触控摇杆
3. 再验证 Windows / 桌面端 Gamepad
4. 最后再接视频流与 Agent 能力

## 回退策略

如果需要临时回退旧页面：

1. 删除或重命名 `turn_on_wheeltec_robot/web/dist`
2. 重启 `web_dashboard_server.py`
3. 服务会自动回退到旧 `web/index.html`
