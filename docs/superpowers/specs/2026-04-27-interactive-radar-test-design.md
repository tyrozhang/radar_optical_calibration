# 交互式雷达终端测试设计文档

## 目标

为 `SmoothTrackingController` 添加一个交互式终端测试入口，允许用户从命令行逐行输入雷达目标坐标（纬度 B、经度 L、高度 H、可选面积），实时观察光电跟踪命令的生成与预测外推过程，以最接近实际情况的方式验证平滑跟踪算法。

## 设计概述

在现有的 `SmoothTrackingControllerTest` 测试类中新增一个独立的 `interactiveTest` main 方法入口。启动后通过 `Scanner` 读取终端输入，将每个输入点作为雷达数据注入 `SmoothTrackingController`，并自动触发完整的预测周期（连续 tick N 次），期间输出完整的 `TrackingCommand` 字段与内部状态。

预测最大次数和 tick 间隔支持启动时可配置。

## 详细设计

### 1. 入口与位置

- **位置**：`SmoothTrackingControllerTest.interactiveTest(String[] args)`
- **运行方式**：
  - IDE 中直接运行 `SmoothTrackingControllerTest.main`，通过参数区分模式（如 `"--interactive"` 触发交互模式）
  - 或命令行：
    ```bash
    mvn compile exec:java \
      -Dexec.mainClass="gds.cloud.module.antidrone.utils.radar.calibration.SmoothTrackingControllerTest" \
      -Dexec.args="--interactive --prediction-count 10 --tick-interval 20"
    ```

### 2. 启动配置

启动时依次提示（可回车使用默认值）：

```
========== 交互式雷达跟踪测试 ==========
预测最大次数 (默认 10): 10
Tick 间隔 ms (默认 20): 20
标定配置文件路径 (默认 calibration_v2.json): calibration_v2.json
========================================
```

- `--prediction-count <N>`：每次收到新雷达点后，自动连续 tick 的最大次数。
- `--tick-interval <ms>`：相邻两次 tick 之间的时间间隔。
- `--config <path>`：标定配置文件路径，找不到则回退到 `defaultConfig()` 并打印警告。

### 3. 输入格式

终端持续提示：

```
输入雷达点（格式: B L H [面积]，例如 39.900 116.400 100 或 39.900 116.400 100 25.5）
输入 q 退出:
> 39.900 116.400 100
```

- **B**：目标纬度（度）
- **L**：目标经度（度）
- **H**：目标高度（米）
- **面积**（可选）：雷达探测目标面积（平方米），省略时传 `null`
- 输入 `q`、`quit`、`exit` 退出程序

解析失败时打印错误提示，不退出，继续等待下一次输入。

### 4. 时间戳策略

- 每个输入点收到时，以 `System.currentTimeMillis()` 作为该雷达点的时间戳。
- 自动 tick 阶段，每次 tick 的时间戳为 `上一个时间戳 + tickInterval`。
- 如果用户在预测阶段中输入新点，立即中断当前预测周期，以新点的当前时间戳重新开始。

### 5. 输出格式（详细模式）

每输入一个点后，程序执行以下输出：

```
========== 雷达点 #1 | 时间戳: 1714212345678 ==========
输入: B=39.9000, L=116.4000, H=100.0, 面积=null

[立即响应]
  时间偏移: 0ms
  azCmd=123.4567°    elCmd=45.6789°
  azGeo=123.4500°    elGeo=45.6700°
  dAz0=0.0000°       dEl0=0.0000°
  dAzRadar=0.0100°   dElRadar=-0.0050°
  range=1523.4m      targetWidth=2.1m   targetHeight=1.5m
  阶段: 立即响应 (predictionCount=0 / max=10)

[预测 1/10]
  时间偏移: 20ms
  azCmd=123.4571°    elCmd=45.6791°
  ...
  阶段: 预测中 (predictionCount=1 / max=10)

[预测 2/10]
  ...

...

[预测 10/10]
  ...
  阶段: 预测完成，等待下一雷达点 (predictionCount=10 / max=10)

等待下一个雷达点...
>
```

输出字段包含 `TrackingCommand` 的全部 12 个字段，以及当前 `predictionCount` 和 `maxPredictionCount`。

### 6. 内部状态与阶段

程序维护并展示以下状态：

| 状态 | 条件 |
|---|---|
| 等待首个雷达点 | `!hasFirstPoint` |
| 轨迹未就绪 | `hasFirstPoint && !tracker.isReady()` |
| 立即响应 | `onRadarData` 调用后的第一次输出 |
| 预测中 | `predictionCount <= maxPredictionCount` |
| 预测完成，等待下一雷达点 | `predictionCount > maxPredictionCount` |

### 7. 配置加载策略

1. 优先加载用户指定的配置文件路径。
2. 如果文件不存在或解析失败，打印警告并回退到测试内置的 `defaultConfig()`（零偏差、固定雷达补偿）。
3. `defaultConfig()` 使用 `StationBLH(39.800, 116.300, 50)`。

### 8. 边界情况处理

| 场景 | 处理 |
|---|---|
| 输入格式错误（字段数不足或类型错误） | 打印 `输入格式错误，期望: B L H [面积]`，继续等待输入 |
| 配置文件不存在 | 打印警告，回退 `defaultConfig()`，继续运行 |
| 首个雷达点到达 | 立即响应，但轨迹未就绪（仅有1个点），不进行预测 |
| 第二个雷达点到达 | 轨迹就绪，开始预测 |
| 预测过程中收到新点 | 立即中断当前周期，重置 `predictionCount`，进入新周期 |
| 用户输入 `q`/`quit`/`exit` | 优雅退出，关闭 Scanner |

## 流程图

```
启动
  │
  ▼
读取启动配置（predictionCount, tickInterval, configPath）
  │
  ▼
初始化 SmoothTrackingController
  │
  ▼
┌──────────────────────────────┐
│  等待终端输入                  │
│  > B L H [面积]               │
│  > q / quit / exit            │
└──────────────────────────────┘
  │
  ├── 输入 q/quit/exit ──→ 退出程序
  │
  ├── 输入格式错误 ──────→ 提示错误，回到等待输入
  │
  └── 输入正确 ──────────→ 生成当前时间戳
                            │
                            ▼
                      调用 onRadarData(timestamp, B, L, H, area)
                            │
                            ▼
                      输出 [立即响应] 详情
                            │
                            ▼
                      ┌──────────────────────────┐
                      │ 循环 i = 1 to maxPredictionCount │
                      │   tick(timestamp + i * tickInterval)   │
                      │   输出 [预测 i/maxPredictionCount] 详情  │
                      └──────────────────────────┘
                            │
                            ▼
                      回到等待终端输入
```

## 风险评估

- **内联在测试类中的弊端**：`SmoothTrackingControllerTest` 同时承担单元测试和交互式测试工具两种职责，未来若交互测试需要扩展（如 CSV 回放、网络输入），会污染测试类。
- **缓解措施**：当前设计将交互逻辑尽量封装在独立方法中，与现有 `testXxx` 方法互不干扰；若未来扩展需求增加，可轻松提取为独立的 `InteractiveRadarTest` 类，迁移成本极低。

## 验收标准

- [ ] 启动后可配置 predictionCount 和 tickInterval
- [ ] 终端可逐行输入 `B L H [面积]`，输入 `q` 退出
- [ ] 每个点输入后自动输出 [立即响应] + N 次 [预测] 的完整字段
- [ ] 首个点不预测（轨迹未就绪），第二个点开始预测
- [ ] 配置文件不存在时回退 defaultConfig() 并打印警告
- [ ] 输入格式错误时不崩溃，提示后继续等待
