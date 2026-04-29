# 项目长期记忆

## 编译运行
- 编译：`javac -encoding UTF-8 -d target/classes -sourcepath src/main/java <文件>`
- 运行：`java -cp target/classes <全类名>`
- 输出目录：`target/classes`（不是 out）

## 架构
- **TrackingCore** — 共享基础类（纯静态方法）：角度归一化、雷达补偿查找、基础角度计算
- **RadarOpticTrackerV2** — 单帧计算器（无状态）：配置加载+调试输出+radarArea处理
  - 对外类型 `RadarOpticTrackerV2.RadarCompensation` / `FixedCompensation` / `SegmentedCompensation` 保持不变
  - 内部委托 TrackingCore 计算
- **SmoothTracker** — 连续跟踪器（有状态）：轨迹预测+丢失检测
  - 使用 `TrackingCore.RadarCompensation`，不再反向依赖 V2
- **TrackerInvoker** — 第三方接入层，管理多个 V2 实例
- **CalibrationValidator** — 标定参数反向验证，使用 V2 计算理论值

## 坐标与角度约定
- 坐标顺序：B L H（纬度 经度 高度）
- 方位角 Az：0~360° 顺时针北起
- 俯仰角 El：上负下正 ±90°（仰视为负，俯视为为正）
- 距离补偿逻辑必须保留，不可将雷达偏差合并到 dAz0

## 当前标定状态
- El RMS 0.23° 优异
- Az RMS 2.56° 存在系统偏差（均值 -1.72°），正在排查
- 已确认几何角计算基准应从 target_blh 切换至 radar_blh
