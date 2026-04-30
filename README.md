# 雷达光电联动纠偏标定系统 (Phase 2)

## 程序目标

本程序用于解决雷达与光电设备联合跟踪目标时的角度偏差问题。通过采集目标的 RTK 真值坐标、雷达上报坐标和光电实测角度，标定出：

1. **光电静态偏差**（ΔAz0, ΔEl0）：光电设备自身的零点偏移
2. **雷达补偿参数**：根据雷达误差分析结果，生成固定补偿或按距离分段补偿参数

标定结果输出为 `calibration_v2.json`，供在线跟踪器实时计算光电驱动指令，实现雷达引导光电的精确联动。

## 主要思路

### 1. 数据采集
使用 RTK 无人机或模拟数据生成器，在多个距离段（0-1km、1-3km、3-5km、5-10km）采集样本。每个样本包含：
- `target_blh`：RTK 真值坐标
- `radar_blh`：雷达上报坐标
- `Az_measured` / `El_measured`：光电实测角度

### 2. 光电静态偏差标定
对每一点计算几何真值角（由 RTK 真值与光电站坐标通过 WGS84 坐标转换得到），与光电实测角对比，加权平均得到零点偏差 ΔAz0 和 ΔEl0。

### 3. 雷达误差分析
将雷达角度误差按距离和高度分段统计，计算各段的均值与标准差。若各段误差差异显著（阈值 0.03°），则采用**分段补偿**；否则采用**固定补偿**。

### 4. 在线跟踪
跟踪器读取 `calibration_v2.json`，对传入的目标坐标计算理想几何角，叠加光电偏差和雷达补偿，输出最终的光电驱动指令（Az_cmd, El_cmd）。

## 文件说明

### Java 源文件

| 文件 | 作用 |
|------|------|
| `algo/CalibrateV2.java` | **标定主程序**。读取标定数据，计算光电静态偏差，调用雷达误差分析，输出 `calibration_v2.json` |
| `algo/RadarErrorAnalyzer.java` | **雷达误差分析器**。按配置的分段（距离/高度）统计误差，决策使用固定补偿或分段补偿，生成分析报告 |
| `algo/Simulator.java` | **模拟数据生成器**。基于真实物理模型生成带一致性的测试数据（32 个样本，均匀覆盖 4 个距离段），输出 `calibration_data_v2.json` |
| `RadarOpticTrackerV2.java` | **在线跟踪器**。加载标定结果，接收目标 BLH 坐标，输出光电驱动指令。支持交互模式和模拟测试模式。**增强版**：新增雷达距离/面积参数输入，输出光电到目标距离和目标尺寸信息 |
| `TrackingCore.java` | **跟踪核心计算逻辑**（共享基础类）。抽取公共逻辑：雷达补偿类型定义、补偿值查找、角度归一化、基础角度计算、目标尺寸计算 |
| `TrackingCommand.java` | **光电跟踪命令**（统一数据结构）。覆盖控制指令、中间量、目标信息、时间戳的完整记录 |
| `TrackerConfigLoader.java` | **标定配置加载器**。从 `calibration_v2.json` 解析标定参数，供跟踪器和控制器共用 |
| `TrackPredictor.java` | **航迹预测器**。基于雷达提供的真实速度数据（velEast/North/Up）进行线性外推，解决雷达上报频率低导致的光电画面跳跃问题 |
| `SmoothTracker.java` | **纯预测器**。管理轨迹滑动窗口（最近 2 个雷达点），基于速度线性外推计算任意时刻的目标位置 |
| `SmoothTrackingController.java` | **联动控制器**。状态机驱动，协调雷达数据到达和光电控制周期 |
| `SmoothTrackingControllerInvoker.java` | **平滑跟踪控制器调用器**（第三方接入层）。提供全局默认实例，支持多实例管理 |
| `TrackerInvoker.java` | **第三方接入调用器**。统一入口，支持多实例管理、幂等注册、并发安全 |
| `model/StationBLH.java` | 站点大地坐标数据模型（B: 纬度°, L: 经度°, H: 高度 m） |
| `model/SegmentConfig.java` | 基础分段配置类（名称、最小/最大边界、包含判断） |
| `model/RadarSegmentConfig.java` | 雷达分段配置类，继承 SegmentConfig，增加补偿值 dAz / dEl |
| `util/CoordinateUtils.java` | WGS84 坐标转换工具：BLH ↔ ECEF ↔ ENU ↔ AzEl，以及 Haversine 球面距离计算 |
| `util/SimpleJsonParser.java` | 简易 JSON 解析器，无第三方依赖，支持对象、数组、字符串、数字、布尔值 |

### 测试文件

| 文件 | 作用 |
|------|------|
| `test/DebugTest.java` / `test/DebugTest2.java` | 调试测试程序 |
| `test/TrackerInvokerTest.java` | TrackerInvoker 单元测试 |
| `test/CalibrationReverseTest.java` | **标定参数反向验证测试**。反向验证标定质量，对比计算值与实测值，输出误差统计 |
| `test/SmoothTrackingControllerTest.java` | 平滑跟踪控制器交互式测试 |

### JSON 文件

| 文件 | 作用 |
|------|------|
| `calibration_data_v2.json` | **标定输入数据**。包含雷达/光电站坐标和 32 个数据点的 RTK 真值、雷达上报、光电实测角度 |
| `calibration_data_v2_all.json` | 扩展标定输入数据（更多样本） |
| `calibration_data_v2_test.json` | 测试用标定输入数据 |
| `calibration_v2.json` | **标定结果输出**。包含光电偏差参数、雷达补偿策略（固定/分段）、质量评估和误差分析报告 |

## 环境要求

- **JDK 17+**（代码使用了 `record` 语法）
- **Maven 3.6+**（推荐 3.9+）
- 系统默认 Java 可能是 1.8，需切换 JAVA_HOME：

```powershell
$env:JAVA_HOME = "D:\java\jdk-17.0.2"
```

## 如何使用

> **注意**：以下所有命令均需在项目根目录下执行。

### 1. 编译打包（Maven）

```powershell
mvn clean package -DskipTests
```

打包产物在 `target/radar-calibration.jar`（fat jar，可直接运行）。

或手动编译（不使用 Maven）：

```bash
javac -encoding UTF-8 -d target/classes src/main/java/com/radar/calibration/**/*.java
```

或如果已提供 jar 包：

```bash
java -jar radar-calibration.jar
```

### 2. 生成模拟标定数据（可选）

如果没有真实采集数据，先用模拟器生成测试数据：

```bash
java -cp target/classes gds.cloud.module.antidrone.utils.radar.calibration.algo.Simulator
```

或使用 jar 包运行（需在项目根目录）：

```bash
java -cp target/radar-calibration.jar gds.cloud.module.antidrone.utils.radar.calibration.algo.Simulator
```

参数说明：
- `-i`：原始数据输入文件（获取雷达/光电站坐标），默认 `calibration_data.json`
- `-o`：生成的 Phase 2 模拟数据输出文件，默认 `calibration_data_v2.json`

### 3. 运行标定程序

```bash
java -cp target/classes gds.cloud.module.antidrone.utils.radar.calibration.algo.CalibrateV2
```

或使用 jar 包：

```bash
java -jar target/radar-calibration.jar
```

参数说明：
- `-i`：标定输入数据文件，默认 `calibration_data_v2.json`
- `-o`：标定结果输出文件，默认 `calibration_v2.json`

程序会输出：
- 光电标定结果（ΔAz0, ΔEl0, RMS）
- 雷达误差按距离分段统计
- 补偿策略决策（固定/分段）
- 最终标定文件 `calibration_v2.json`

### 4. 在线跟踪器（交互模式）

加载标定结果，实时接收目标坐标并计算光电驱动指令。

**启动交互模式**：

```bash
java -cp target/classes gds.cloud.module.antidrone.utils.radar.calibration.RadarOpticTrackerV2
```

或使用 jar 包：

```bash
java -cp target/radar-calibration.jar gds.cloud.module.antidrone.utils.radar.calibration.RadarOpticTrackerV2
```

可选参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-c <path>` | 标定配置文件路径 | `calibration_v2.json` |
| `-s` / `--simulate` | 运行模拟跟踪测试（非交互） | — |

**输入格式说明**：

| 参数 | 位置 | 类型 | 说明 |
|------|------|------|------|
| `B` | 第1个 | double | 目标纬度（度） |
| `L` | 第2个 | double | 目标经度（度） |
| `H` | 第3个 | double | 目标高度（米） |
| `radarRange` | 第4个 | double (可选) | 雷达探测的目标斜距（米） |
| `radarArea` | 第5个 | double (可选) | 雷达探测的目标面积（平方米） |

**交互操作**：

```
[*] Phase 2 跟踪器已就绪，等待目标数据...
    输入格式: B L H [radarRange] [radarArea]
    示例: 39.905 116.408 100 5000 2.5
    输入 'q' 退出

39.9050 116.4080 100.0 5000 2.5  ← 输入目标坐标 + 雷达参数（可选）
    → Az=3002  El=2594
      光电距目标: 114.06m | 目标宽: 1.58m | 目标高: 1.58m

q                                  ← 输入 q 退出
```

**输出字段说明**：

| 字段 | 含义 |
|------|------|
| `Az` / `El` | 最终光电驱动指令（整数编码值 = 角度 ×100） |
| `opticalToTargetRange` | 光电设备到目标的距离（米） |
| `targetWidth` | 目标宽（米）：有 radarArea 时为 √radarArea，否则为 0.35m |
| `targetHeight` | 目标高（米）：有 radarArea 时为 √radarArea，否则为 0.01m |

**模拟跟踪测试**（自动生成 10 帧轨迹，用于验证）：

```bash
java -cp target/classes gds.cloud.module.antidrone.utils.radar.calibration.RadarOpticTrackerV2 -s
```

或使用 jar 包：

```bash
java -cp target/radar-calibration.jar gds.cloud.module.antidrone.utils.radar.calibration.RadarOpticTrackerV2 -s
```

### 5. 使用标定结果文件

`calibration_v2.json` 中的 `parameters` 段包含所有标定参数，可直接被跟踪器或其他系统集成：

```json
{
  "parameters": {
    "radar_blh": { "B": 39.9042, "L": 116.4074, "H": 50.0 },
    "optical_blh": { "B": 39.9043, "L": 116.4075, "H": 51.0 },
    "dAz0": 0.003378,
    "dEl0": 0.000401,
    "radar_compensation_type": "fixed",
    "dAz_radar": 0.055442,
    "dEl_radar": 0.02155
  }
}
```

## 预测模式联动光电联动模式

### 功能说明

预测模式联动光电联动模式用于解决**雷达上报频率低、光电控制频率高**时的跟踪连续性问题。雷达通常以较低频率（如 0.5Hz，每 2 秒一个点）上报目标坐标，而光电伺服需要更高频率（如 50Hz，每 20ms 一次）的指令才能平滑跟踪。该模式通过**速度外推预测**填补雷达点之间的空白，实现光电对目标的连续、平滑跟踪。

### 核心组件

| 组件 | 文件 | 职责 |
|------|------|------|
| `TrackPredictor` | `TrackPredictor.java` | **真实速度航迹预测器**。利用雷达提供的真实速度数据（velEast/North/Up）或位置差分估算速度，基于 UTC 时间戳进行线性外推，支持机动检测 |
| `SmoothTracker` | `SmoothTracker.java` | **纯预测器**。管理轨迹滑动窗口（最近 2 个雷达点），基于速度线性外推计算任意时刻的目标位置 |
| `SmoothTrackingController` | `SmoothTrackingController.java` | **联动控制器**。状态机驱动，协调雷达数据到达和光电控制周期 |
| `SmoothTrackingControllerInvoker` | `SmoothTrackingControllerInvoker.java` | **第三方接入调用器**。全局默认实例 + 多实例管理，支持并发安全 |

### 工作流程

```
雷达数据到达（低频）
    │
    ▼
┌─────────────────────────────────────┐
│  立即执行：光电指向雷达上报坐标      │
│  （应用标定补偿：雷达补偿 + 光电零偏）│
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│  预测阶段（N 次光电 tick）           │
│  基于最近 2 个雷达点计算速度         │
│  线性外推目标位置 → 计算光电角度     │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│  等待下一雷达点                      │
│  光电保持最后位置不动                │
└─────────────────────────────────────┘
    │
    ▼
  下一雷达点到达 → 回到第一步
```

### 状态机说明

| 状态 | 触发条件 | 行为 |
|------|----------|------|
| 等待首个雷达点 | 控制器初始化后 | 不发命令，光电保持当前位置 |
| 立即执行 | 收到新雷达点 | 光电立即指向该雷达坐标（经标定补偿） |
| 预测阶段 | 光电 tick 触发，且预测次数 < N | 基于速度外推计算目标位置，驱动光电 |
| 等待 | 预测次数达到 N | 不发命令，光电保持最后位置 |

**关键点**：

- **第 1 个雷达点**：只存储，不联动（速度未知，无法外推）
- **第 2 个雷达点起**：轨迹就绪，开始联动（立即执行 + 预测）
- **滑动窗口**：始终保留最近 2 个雷达点，自动丢弃旧点
- **内插支持**：若查询时刻位于 t1 和 t2 之间，自动线性内插（应对时延补偿场景）

### 配置参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `maxPredictionCount` | 每次新雷达点后最多预测次数（控制预测阶段长度） | 10 |
| `predictionIntervalMs` | 光电控制周期（毫秒，即 tick 间隔） | 20（50Hz） |
| `totalDelayMs` | 雷达到光电的总时延（毫秒，用于时延补偿） | 0 |

### API 使用示例

```java
import gds.cloud.module.antidrone.utils.radar.calibration.SmoothTrackingController;
import gds.cloud.module.antidrone.utils.radar.calibration.TrackingCommand;

// 1. 创建控制器（从标定配置文件）
SmoothTrackingController controller = new SmoothTrackingController(
    "calibration_v2.json",
    new SmoothTrackingController.ServoController() {
        @Override
        public void moveTo(TrackingCommand cmd) {
            // 驱动光电伺服到指定角度
            double az = cmd.azCmd();  // 方位指令（度）
            double el = cmd.elCmd();  // 俯仰指令（度）
            servo.move(az, el);
        }
        @Override
        public void stop() {
            servo.stop();
        }
    },
    10,   // 最多预测 10 次
    20    // 光电控制周期 20ms（50Hz）
);

// 2. 启动控制循环
controller.start();

// 3. 雷达数据到达时调用（在雷达数据回调中）
controller.onRadarData(timestamp, b, l, h);        // 无面积
controller.onRadarData(timestamp, b, l, h, area);  // 有面积

// 4. 切换目标时重置
controller.reset();

// 5. 停止
controller.stop();
```

### 运行测试

交互式测试（手动输入雷达点，观察预测行为）：

```bash
java -cp "target/classes;target/test-classes" gds.cloud.module.antidrone.utils.radar.calibration.SmoothTrackingControllerTest --interactive --prediction-count 10 --tick-interval 500
```

参数说明：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--interactive` | 启用交互模式（手动输入雷达点） | — |
| `--prediction-count` | 每次新点后最多预测次数 | 10 |
| `--tick-interval` | 预测间隔时间（毫秒） | 50 |

交互模式下输入格式：

```
B L H [radarArea]
```

示例：

```
39.900 116.400 100        ← 无目标面积
39.900 116.400 100 25.5   ← 有目标面积（25.5 平方米）
```

### 设计特点

1. **不依赖雷达周期假设**：以实际雷达点到达为触发，而非假设固定周期
2. **以预测次数控制平滑长度**：不硬编码时间窗口，适应不同雷达频率
3. **预测完成后自动停止**：不发冗余命令，光电保持最后位置，降低伺服抖动
4. **纯几何外推**：基于大地坐标（BLH）的速度外推，避免角度域的奇点问题（如方位角 360°/0° 跳变）
5. **时延补偿支持**：通过 `totalDelayMs` 参数补偿雷达到光电的通信和处理时延

## 光电联动接口数据增强

### 功能说明

跟踪器输出新增光电联动所需的距离和目标尺寸信息，便于光电设备进行精确定位和尺寸补偿。

### API 调用示例

```java
// 无雷达参数（使用默认值）
TrackingCommand cmd1 = tracker.compute(39.905, 116.408, 100);
// cmd1.targetWidth() = 0.35
// cmd1.targetHeight() = 0.01

// 有雷达参数
TrackingCommand cmd2 = tracker.compute(39.905, 116.408, 100, 5000.0, 2.5);
// cmd2.opticalToTargetRange() = 114.06  (光电到目标距离，米)
// cmd2.targetWidth() = 1.58            (√2.5，保留2位小数)
// cmd2.targetHeight() = 1.58
```

### 常量定义

```java
public static final double DEFAULT_TARGET_WIDTH = 0.35;   // 米
public static final double DEFAULT_TARGET_HEIGHT = 0.01;   // 米
```

## 第三方接入（TrackerInvoker）

### 概述

`TrackerInvoker` 是跟踪程序的统一调用入口，供第三方系统集成接入。支持多实例管理、并发安全、幂等注册。

### 文件

| 文件 | 作用 |
|------|------|
| `TrackerInvoker.java` | 跟踪程序调用器（第三方接入层） |
| `TrackerInvokerTest.java` | 单元测试程序 |

### 设计要点

- **全局默认实例**：`TrackerInvoker.getDefault()` 获取单例
- **并发安全**：`ConcurrentHashMap` 管理 tracker 实例
- **幂等注册**：`register()` 重复调用不重建，`registerForce()` 强制覆盖
- **明确的错误处理**：`compute()` 对未注册 key 抛出 `IllegalArgumentException`

### API 清单

| 方法 | 说明 |
|------|------|
| `getDefault()` | 获取全局默认实例 |
| `register(key, configPath)` | 幂等注册 tracker（已存在则复用） |
| `registerForce(key, configPath)` | 强制重建 tracker（覆盖已有实例） |
| `get(key)` | 获取 tracker，返回 `Optional<RadarOpticTrackerV2>` |
| `contains(key)` | 检查 tracker 是否已注册 |
| `remove(key)` | 移除 tracker 实例 |
| `compute(key, B, L, H, radarRange, radarArea)` | 计算光电驱动指令 |
| `listKeys()` | 列出所有已注册的 key |
| `clear()` | 清空所有 tracker 实例 |

### 使用示例

```java
import com.radar.calibration.TrackerInvoker;
import com.radar.calibration.RadarOpticTrackerV2;

// 1. 获取全局实例
TrackerInvoker invoker = TrackerInvoker.getDefault();

// 2. 注册 tracker
invoker.register("tracker-001", "calibration_v2.json");

// 3. 计算光电驱动指令
RadarOpticTrackerV2.TrackingCommand cmd = invoker.compute(
    "tracker-001",           // key
    39.905,                 // 目标纬度 B
    116.408,                // 目标经度 L
    100.0,                  // 目标高度 H（米）
    5000.0,                 // 雷达斜距（米，可为 null）
    2.5                     // 雷达目标面积（平方米，可为 null）
);

// 4. 读取结果
double azCmd = cmd.azCmd();          // 方位指令（十进制度）
double elCmd = cmd.elCmd();          // 俯仰指令（十进制度）
double range = cmd.opticalToTargetRange();  // 光电到目标距离（米）
double width = cmd.targetWidth();    // 目标宽（米）
double height = cmd.targetHeight();  // 目标高（米）
```

### 编译与运行

```bash
# 编译
javac -encoding UTF-8 -d target/classes -sourcepath src/main/java \
    src/main/java/com/radar/calibration/TrackerInvokerTest.java

# 运行测试
java -cp target/classes com.radar.calibration.TrackerInvokerTest
```

---

## 标定参数验证（CalibrationReverseTest）

### 功能说明

`CalibrationReverseTest` 用于**反向验证标定参数的质量**。它读取 `calibration_data_v2.json` 中的雷达 BLH 数据点，使用 `RadarOpticTrackerV2` 计算理论 Az/El，与实测 `Az_measured`/`El_measured` 对比，输出误差统计和评估结论。

### 使用场景

1. **验证新标定参数**：生成 `calibration_v2.json` 后，确认标定质量是否达标
2. **对比不同数据集**：用同一套参数验证不同批次采集的数据
3. **持续监控**：集成到标定流程，自动验证每次标定结果

### 运行方式

```bash
# 默认运行（使用默认文件路径）
java -cp "target/classes;target/test-classes" gds.cloud.module.antidrone.utils.radar.calibration.CalibrationReverseTest

# 指定数据文件、配置文件、使用哪类数据。其中 -s (值为 target_blh / radar_blh, 默认 target_blh)
java -cp "target/classes;target/test-classes" gds.cloud.module.antidrone.utils.radar.calibration.CalibrationReverseTest -d calibration_data_v2.json -c calibration_v2.json -s radar_blh
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-d <path>` | 原始标定数据文件路径 | `calibration_data_v2.json` |
| `-c <path>` | 标定参数配置文件路径 | `calibration_v2.json` |
| `-s <type>` | 信号类型（`target_blh` 或 `radar_blh`） | `target_blh` |

### 输出解读

程序输出三部分内容：

**1. 逐点对比表**
```
序号   B            L            H            Az_实测      Az_计算      Az_误差      El_实测      El_计算      El_误差      Az_几何      El_几何
1      39.9042      116.4074     50.0000      8.9200       6.9300       -1.9900      -6.4500      -6.7779      -0.3279      ...          ...
...
```

**2. 误差统计**
```
【方位角 (Az) 误差】
  均值:     -1.7217°      ← 系统性偏差
  标准差:   1.8908°       ← 离散程度
  RMS:      2.5572°       ← 综合误差指标（核心指标）
  最大绝对值: 4.9320°
```

**3. 质量评估结论**
- ✓ **优秀**：RMS < 0.5°
- ✓ **良好**：RMS < 1.0°
- △ **一般**：RMS < 2.0°
- ✗ **较差**：RMS ≥ 2.0°（建议重新标定）

### 关键指标关注

| 指标 | 说明 | 建议阈值 |
|------|------|----------|
| RMS | 均方根误差，综合反映准确度 | < 1.0° |
| 均值 | 系统性偏差 | 接近 0° |
| 标准差 | 随机误差大小 | < 1.0° |
| 最大绝对值 | 最坏情况误差 | < 3.0° |
