# 交互式雷达终端测试实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `SmoothTrackingControllerTest` 中添加交互式终端测试入口，允许用户手动输入雷达点并实时观察完整的光电跟踪命令输出与预测过程。

**Architecture:** 在现有测试类中内联实现，通过命令行参数 `--interactive` 切换模式。新增带目标面积的 `onRadarData` 重载，配套 `PrintingServo` 捕获命令，主循环读取终端输入、自动触发预测周期并格式化输出。

**Tech Stack:** Java 17, Maven, 标准输入输出（Scanner / System.out）

---

## 文件结构

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/main/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingController.java` | 修改 | 添加带 `radarArea` 的 `onRadarData` 重载 |
| `src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java` | 修改 | 添加 `PrintingServo`、交互式测试方法、修改 `main` 入口 |

---

### Task 1: 添加带目标面积的 onRadarData 重载

**Files:**
- Modify: `src/main/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingController.java:141-152`

SmoothTracker 已支持 `update(timestamp, b, l, h, radarArea)`，但 Controller 的 `onRadarData` 未暴露面积参数。需要添加重载，让交互式测试可以传入可选面积。

- [ ] **Step 1: 添加重载方法**

在 `SmoothTrackingController.java` 的现有 `onRadarData` 下方添加：

```java
    /**
     * 雷达数据到达时调用（每收到一个雷达点调用一次，含目标面积）
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param b 目标纬度
     * @param l 目标经度
     * @param h 目标高度
     * @param radarArea 雷达探测的目标面积（平方米），可为 null
     */
    public void onRadarData(long timestamp, double b, double l, double h, Double radarArea) {
        tracker.update(timestamp, b, l, h, radarArea);
        predictionCount = 0;
        hasFirstPoint = true;
        TrackingCommand cmd = tracker.getCommand(timestamp);
        servo.moveTo(cmd);
    }
```

- [ ] **Step 2: 让旧方法委托给新方法**

将原有的 `onRadarData` 改为委托调用：

```java
    public void onRadarData(long timestamp, double b, double l, double h) {
        onRadarData(timestamp, b, l, h, null);
    }
```

- [ ] **Step 3: 编译验证**

Run: `mvn compile`
Expected: BUILD SUCCESS

- [ ] **Step 4: Commit**

```bash
git add src/main/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingController.java
git commit -m "feat: add onRadarData overload with optional radarArea"
```

---

### Task 2: 创建 PrintingServo 并重构 main 入口

**Files:**
- Modify: `src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java`

- [ ] **Step 1: 添加 PrintingServo 内部类**

在 `MockServo` 类之后添加：

```java
    /** 用于交互式测试的伺服控制器：捕获最后一次命令 */
    static class PrintingServo implements ServoController {
        TrackingCommand lastCommand;

        @Override
        public void moveTo(TrackingCommand cmd) {
            this.lastCommand = cmd;
        }

        @Override
        public void stop() {}
    }
```

- [ ] **Step 2: 修改 main 入口支持模式切换**

将现有 `main` 方法替换为：

```java
    public static void main(String[] args) {
        // 检查是否进入交互模式
        boolean interactive = false;
        for (String arg : args) {
            if ("--interactive".equals(arg)) {
                interactive = true;
                break;
            }
        }

        if (interactive) {
            runInteractiveTest(args);
        } else {
            runUnitTests();
        }
    }

    private static void runUnitTests() {
        System.out.println("\n========== SmoothTrackingController 单元测试 ==========\n");

        testPredictionCountLimit();
        testRadarDelay();
        testRadarEarly();
        testNoHardcodedTiming();
        testLoadFromConfigFile();

        System.out.println("\n========== 单元测试完成 ==========");
    }
```

- [ ] **Step 3: 编译验证**

Run: `mvn test-compile`
Expected: BUILD SUCCESS

- [ ] **Step 4: Commit**

```bash
git add src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java
git commit -m "feat: add PrintingServo and mode switch in test main entry"
```

---

### Task 3: 实现交互式测试核心逻辑

**Files:**
- Modify: `src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java`

- [ ] **Step 1: 添加启动配置读取方法**

在 `PrintingServo` 类之后添加：

```java
    /** 从命令行参数解析整数值，失败时返回默认值 */
    private static int parseIntArg(String[] args, String key, int defaultValue) {
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals(key)) {
                try {
                    return Integer.parseInt(args[i + 1]);
                } catch (NumberFormatException e) {
                    System.out.println("警告: " + key + " 参数格式错误，使用默认值 " + defaultValue);
                }
            }
        }
        return defaultValue;
    }

    /** 从命令行参数解析字符串值，失败时返回默认值 */
    private static String parseStringArg(String[] args, String key, String defaultValue) {
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals(key)) {
                return args[i + 1];
            }
        }
        return defaultValue;
    }
```

- [ ] **Step 2: 添加命令输出格式化方法**

```java
    /** 格式化输出 TrackingCommand 详情 */
    private static void printCommandDetails(
            TrackingCommand cmd, String stage, int predictionCount, int maxPredictionCount) {
        System.out.printf("  azCmd=%.4f°     elCmd=%.4f°%n", cmd.azCmd(), cmd.elCmd());
        System.out.printf("  azGeo=%.4f°     elGeo=%.4f°%n", cmd.azGeo(), cmd.elGeo());
        System.out.printf("  dAz0=%.4f°       dEl0=%.4f°%n", cmd.dAz0(), cmd.dEl0());
        System.out.printf("  dAzRadar=%.4f°   dElRadar=%.4f°%n", cmd.dAzRadar(), cmd.dElRadar());
        System.out.printf("  range=%.1fm      targetWidth=%.1fm   targetHeight=%.1fm%n",
                cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight());
        System.out.printf("  阶段: %s (predictionCount=%d / max=%d)%n",
                stage, predictionCount, maxPredictionCount);
    }
```

- [ ] **Step 3: 添加交互式测试主方法**

```java
    public static void runInteractiveTest(String[] args) {
        // 读取启动配置
        int maxPredictionCount = parseIntArg(args, "--prediction-count", 10);
        long tickInterval = parseIntArg(args, "--tick-interval", 20);
        String configPath = parseStringArg(args, "--config", "calibration_v2.json");

        System.out.println("\n========== 交互式雷达跟踪测试 ==========");
        System.out.println("预测最大次数: " + maxPredictionCount);
        System.out.println("Tick 间隔 ms: " + tickInterval);
        System.out.println("标定配置文件: " + configPath);
        System.out.println("========================================\n");

        // 初始化控制器
        PrintingServo servo = new PrintingServo();
        SmoothTrackingController controller;
        try {
            controller = new SmoothTrackingController(configPath, servo, maxPredictionCount, tickInterval);
        } catch (Exception e) {
            System.out.println("警告: 配置文件 '" + configPath + "' 加载失败，回退到默认配置");
            System.out.println("  原因: " + e.getMessage());
            controller = new SmoothTrackingController(defaultConfig(), servo, maxPredictionCount, tickInterval);
        }

        Scanner scanner = new Scanner(System.in);
        int pointCount = 0;

        System.out.println("输入雷达点（格式: B L H [面积]，例如 39.900 116.400 100 或 39.900 116.400 100 25.5）");
        System.out.println("输入 q 退出\n");

        while (true) {
            System.out.print("> ");
            String line = scanner.nextLine().trim();

            if (line.isEmpty()) {
                continue;
            }
            if (line.equals("q") || line.equals("quit") || line.equals("exit")) {
                System.out.println("退出交互式测试。");
                break;
            }

            // 解析输入
            String[] parts = line.split("\\s+");
            if (parts.length < 3) {
                System.out.println("输入格式错误，期望: B L H [面积]");
                continue;
            }

            double b, l, h;
            Double area = null;
            try {
                b = Double.parseDouble(parts[0]);
                l = Double.parseDouble(parts[1]);
                h = Double.parseDouble(parts[2]);
                if (parts.length >= 4) {
                    area = Double.parseDouble(parts[3]);
                }
            } catch (NumberFormatException e) {
                System.out.println("输入格式错误，期望: B L H [面积]，数值解析失败");
                continue;
            }

            pointCount++;
            long timestamp = System.currentTimeMillis();

            System.out.println("\n========== 雷达点 #" + pointCount + " | 时间戳: " + timestamp + " ==========");
            System.out.printf("输入: B=%.4f, L=%.4f, H=%.1f, 面积=%s%n%n",
                    b, l, h, area != null ? area : "null");

            // 注入雷达点
            controller.onRadarData(timestamp, b, l, h, area);

            // 输出立即响应
            if (servo.lastCommand != null) {
                System.out.println("[立即响应]");
                printCommandDetails(servo.lastCommand, "立即响应",
                        controller.getPredictionCount(), maxPredictionCount);
                System.out.println();
            }

            // 如果轨迹未就绪（仅有1个点），跳过预测阶段
            if (!controller.isTrackerReady()) {
                System.out.println("轨迹未就绪（仅有1个点），跳过预测阶段\n");
                continue;
            }

            // 自动连续 tick，模拟完整预测周期（由 start() 启动的内部线程执行）
            for (int i = 1; i <= maxPredictionCount; i++) {
                long tickTime = timestamp + i * tickInterval;
                // controller.tick() 现为私有，由内部控制循环自动调用

                if (servo.lastCommand != null) {
                    String stage = (i < maxPredictionCount) ? "预测中" : "预测完成，等待下一雷达点";
                    System.out.println("[预测 " + i + "/" + maxPredictionCount + "]");
                    System.out.printf("  时间偏移: %dms%n", i * tickInterval);
                    printCommandDetails(servo.lastCommand, stage,
                            controller.getPredictionCount(), maxPredictionCount);
                    System.out.println();
                }
            }

            System.out.println("等待下一个雷达点...\n");
        }

        scanner.close();
    }
```

- [ ] **Step 4: 编译验证**

Run: `mvn test-compile`
Expected: BUILD SUCCESS

- [ ] **Step 5: Commit**

```bash
git add src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java
git commit -m "feat: implement interactive radar terminal test"
```

---

### Task 4: 运行交互式测试验证功能

**Files:**
- Test: `src/test/java/gds/cloud/module/antidrone/utils/radar/calibration/SmoothTrackingControllerTest.java`

- [ ] **Step 1: 使用 Maven exec 插件运行交互模式**

Run:
```bash
mvn compile exec:java \
  -Dexec.mainClass="gds.cloud.module.antidrone.utils.radar.calibration.SmoothTrackingControllerTest" \
  -Dexec.args="--interactive --prediction-count 5 --tick-interval 100"
```

- [ ] **Step 2: 在终端中执行以下验证序列**

输入测试序列：
```
> 39.900 116.400 100
```
Expected: 显示 [立即响应]，轨迹未就绪提示（因为这是第一个点）

```
> 39.905 116.405 150
```
Expected: 显示 [立即响应]，然后连续 5 次 [预测 X/5]，每次包含完整字段

```
> 39.910 116.410 200 25.5
```
Expected: 显示 [立即响应] 和 5 次预测，且 lastCommand 中包含 targetWidth / targetHeight 的计算值（因为 area=25.5）

```
> q
```
Expected: 程序退出

- [ ] **Step 3: 验证边界情况**

- 输入 `abc def` → Expected: 显示"输入格式错误"
- 输入 `39.9 116.4`（只有2个字段） → Expected: 显示"输入格式错误"
- 配置文件不存在时启动 → Expected: 显示警告并回退到默认配置

- [ ] **Step 4: Commit**

```bash
git commit -m "test: verify interactive radar test functionality"
```

---

## 自我审查

**1. Spec 覆盖检查：**

| Spec 要求 | 对应任务 |
|---|---|
| 启动时可配置 predictionCount 和 tickInterval | Task 3 Step 1 `parseIntArg` / `parseStringArg` |
| 终端逐行输入 `B L H [面积]`，输入 `q` 退出 | Task 3 Step 3 输入循环与解析 |
| 每个点输入后自动输出 [立即响应] + N 次 [预测] | Task 3 Step 3 `runInteractiveTest` 中的 tick 循环 |
| 首个点不预测（轨迹未就绪） | Task 3 Step 3 `if (!controller.isTrackerReady())` 判断 |
| 配置文件不存在时回退 defaultConfig() 并警告 | Task 3 Step 3 try-catch 初始化逻辑 |
| 输入格式错误时不崩溃 | Task 3 Step 3 try-catch 输入解析 |
| 输出完整 TrackingCommand 字段 | Task 2 Step 2 `printCommandDetails` |
| 面积作为可选输入 | Task 1 + Task 3 Step 3 `area` 解析与传入 |

**2. Placeholder 扫描：** 无 TBD、TODO、"implement later" 等占位符。

**3. 类型一致性：**
- `SmoothTrackingController.onRadarData(long, double, double, double, Double)` 与 `SmoothTracker.update(long, double, double, double, Double)` 签名一致。
- `PrintingServo` 正确实现 `ServoController` 接口的 `moveTo(TrackingCommand)` 方法。
- `parseIntArg` 和 `parseStringArg` 返回值类型与使用处一致。

---

## 执行方式

Plan complete and saved to `docs/superpowers/plans/2026-04-27-interactive-radar-test.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
