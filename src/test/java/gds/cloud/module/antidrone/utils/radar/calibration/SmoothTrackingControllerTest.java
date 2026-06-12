package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.SmoothTrackingController.ServoController;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;

public class SmoothTrackingControllerTest {

    static class MockServo implements ServoController {
        int moveCount = 0;
        TrackingCommand lastCommand;
        java.util.List<TrackingCommand> commands = new java.util.ArrayList<>();

        @Override
        public void moveTo(TrackingCommand cmd) {
            moveCount++;
            lastCommand = cmd;
            commands.add(cmd);
        }

        @Override
        public void stop() {}
    }

    static class PrintingServo implements ServoController {
        SmoothTrackingController controller;
        long tickInterval;
        int maxPredictionCount;
        TrackingCommand lastCommand;

        @Override
        public void moveTo(TrackingCommand cmd) {
            this.lastCommand = cmd;
            if (controller == null) {
                return;
            }

            int count = controller.getPredictionCount();
            String stage;
            long offset;

            if (count == 0) {
                stage = "立即响应";
                offset = 0;
            } else if (count < maxPredictionCount) {
                stage = "预测中";
                offset = count * tickInterval;
            } else {
                stage = "预测完成，等待下一雷达点";
                offset = count * tickInterval;
            }

            if (count == 0) {
                System.out.println("\n[立即响应]");
            } else {
                System.out.println("\n[预测 " + count + "/" + maxPredictionCount + "]");
            }
            System.out.printf("  时间偏移: %dms%n", offset);
            printCommandDetails(cmd, stage, count, maxPredictionCount);
        }

        @Override
        public void stop() {}
    }

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

    private static String parseStringArg(String[] args, String key, String defaultValue) {
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals(key)) {
                return args[i + 1];
            }
        }
        return defaultValue;
    }

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

    /** 默认测试配置（零偏差、固定雷达补偿） */
    static RadarOpticTrackerV2.Config defaultConfig() {
        return new RadarOpticTrackerV2.Config(
            new StationBLH(39.800, 116.300, 50),
            0, 0,
            new RadarOpticTrackerV2.FixedCompensation(0, 0),
            0L
        );
    }

    public static void main(String[] args) {
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
        testConstruction();
        testImmediateResponseOnRadarData();
        testTrackerReadyState();
        testStateMachineTransitions();
        testReset();
        testAlgorithmConsistencyWithRadarOpticTrackerV2();
        System.out.println("\n========== 单元测试完成 ==========");
    }

    public static void runInteractiveTest(String[] args) {
        int maxPredictionCount = parseIntArg(args, "--prediction-count", 10);
        long tickInterval = parseIntArg(args, "--tick-interval", 20);
        String configPath = parseStringArg(args, "--config", "calibration_v2.json");

        System.out.println("\n========== 交互式雷达跟踪测试 ==========");
        System.out.println("预测最大次数: " + maxPredictionCount);
        System.out.println("Tick 间隔 ms: " + tickInterval);
        System.out.println("标定配置文件: " + configPath);
        System.out.println("========================================\n");

        PrintingServo servo = new PrintingServo();
        SmoothTrackingController controller;
        try {
            controller = SmoothTrackingControllerInvoker.getDefault()
                    .register("interactive", configPath, servo, maxPredictionCount, tickInterval);
        } catch (Exception e) {
            System.out.println("警告: 配置文件 '" + configPath + "' 加载失败，回退到默认配置");
            System.out.println("  原因: " + e.getMessage());
            controller = SmoothTrackingControllerInvoker.getDefault()
                    .register("interactive", defaultConfig(), servo, maxPredictionCount, tickInterval);
        }

        servo.controller = controller;
        servo.tickInterval = tickInterval;
        servo.maxPredictionCount = maxPredictionCount;

        controller.start();

        java.util.Scanner scanner = new java.util.Scanner(System.in);
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
            System.out.printf("输入: B=%.4f, L=%.4f, H=%.1f, 面积=%s%n",
                    b, l, h, area != null ? area : "null");

            controller.onRadarData(timestamp, b, l, h, area);

            if (!controller.isTrackerReady()) {
                System.out.println("轨迹未就绪（仅有1个点），跳过预测阶段");
            }
        }

        controller.stop();
        scanner.close();
    }

    private static void testConstruction() {
        System.out.println("【测试1】构造器多样性");
        System.out.println("-".repeat(50));

        MockServo servo = new MockServo();
        boolean passed = true;

        // 1. 全参数构造
        try {
            SmoothTrackingController c1 = new SmoothTrackingController(
                new StationBLH(39.8, 116.3, 50), 0, 0,
                new RadarOpticTrackerV2.FixedCompensation(0, 0),
                servo, 5, 100, 0L);
        } catch (Exception e) { passed = false; }

        // 2. 默认参数构造
        try {
            SmoothTrackingController c2 = new SmoothTrackingController(
                new StationBLH(39.8, 116.3, 50), 0, 0,
                new RadarOpticTrackerV2.FixedCompensation(0, 0),
                servo);
        } catch (Exception e) { passed = false; }

        // 3. Config 构造
        try {
            SmoothTrackingController c3 = new SmoothTrackingController(
                defaultConfig(), servo, 5, 100);
        } catch (Exception e) { passed = false; }

        // 4. 配置文件构造（可能失败，允许跳过）
        try {
            SmoothTrackingController c4 = new SmoothTrackingController(
                "calibration_v2.json", servo, 5, 100);
        } catch (Exception e) {
            System.out.println("  配置文件构造 skipped: " + e.getMessage());
        }

        System.out.println(passed ? "✓ 通过" : "✗ 失败");
    }

    private static void testImmediateResponseOnRadarData() {
        System.out.println("\n【测试2】onRadarData 立即触发伺服命令");
        System.out.println("-".repeat(50));

        MockServo servo = new MockServo();
        SmoothTrackingController ctrl = new SmoothTrackingController(
            defaultConfig(), servo, 5, 100);

        // 第一个点：轨迹未就绪，不联动
        ctrl.onRadarData(1000, 39.900, 116.400, 100);
        boolean p1 = servo.moveCount == 0 && servo.lastCommand == null;

        // 第二个点：轨迹就绪，立即响应，命令非空
        ctrl.onRadarData(2000, 39.905, 116.405, 150);
        boolean p2 = servo.moveCount == 1 && servo.lastCommand != null;

        System.out.println((p1 && p2) ? "✓ 通过" : "✗ 失败");
    }

    private static void testTrackerReadyState() {
        System.out.println("\n【测试3】轨迹就绪状态");
        System.out.println("-".repeat(50));

        MockServo servo = new MockServo();
        SmoothTrackingController ctrl = new SmoothTrackingController(
            defaultConfig(), servo, 5, 100);

        // 0 个点
        boolean s0 = ctrl.isWaitingFirstPoint() && !ctrl.isTrackerReady();

        // 1 个点：已有首点，但速度未知
        ctrl.onRadarData(1000, 39.900, 116.400, 100);
        boolean s1 = !ctrl.isWaitingFirstPoint() && !ctrl.isTrackerReady();

        // 2 个点：可计算速度，tracker 就绪
        ctrl.onRadarData(2000, 39.905, 116.405, 150);
        boolean s2 = !ctrl.isWaitingFirstPoint() && ctrl.isTrackerReady();

        System.out.println((s0 && s1 && s2) ? "✓ 通过" : "✗ 失败");
    }

    private static void testStateMachineTransitions() {
        System.out.println("\n【测试4】状态机转换（纯同步，不调用 tick）");
        System.out.println("-".repeat(50));

        MockServo servo = new MockServo();
        SmoothTrackingController ctrl = new SmoothTrackingController(
            defaultConfig(), servo, 5, 100);

        // 初始：等待首个点
        boolean s0 = ctrl.isWaitingFirstPoint()
                && !ctrl.isInPredictionPhase()
                && !ctrl.isWaitingNextPoint();

        // 第 1 个点：hasFirst，predictionCount=0 <= max，属于预测阶段（虽然 tracker 未就绪）
        ctrl.onRadarData(1000, 39.900, 116.400, 100);
        boolean s1 = !ctrl.isWaitingFirstPoint()
                && ctrl.isInPredictionPhase()
                && !ctrl.isWaitingNextPoint();

        // 第 2 个点：predictionCount 被重置为 0，仍在预测阶段
        ctrl.onRadarData(2000, 39.905, 116.405, 150);
        boolean s2 = !ctrl.isWaitingFirstPoint()
                && ctrl.isInPredictionPhase()
                && !ctrl.isWaitingNextPoint();

        System.out.println((s0 && s1 && s2) ? "✓ 通过" : "✗ 失败");
    }

    private static void testReset() {
        System.out.println("\n【测试5】reset 清除状态");
        System.out.println("-".repeat(50));

        MockServo servo = new MockServo();
        SmoothTrackingController ctrl = new SmoothTrackingController(
            defaultConfig(), servo, 5, 100);

        ctrl.onRadarData(1000, 39.900, 116.400, 100);
        ctrl.onRadarData(2000, 39.905, 116.405, 150);

        ctrl.reset();

        boolean passed = ctrl.isWaitingFirstPoint()
                && !ctrl.isTrackerReady()
                && ctrl.getPredictionCount() == 0;

        System.out.println(passed ? "✓ 通过" : "✗ 失败");
    }

    /** 捕获最近一次 moveTo 命令的伺服 */
    static class CapturingServo implements ServoController {
        TrackingCommand lastCommand;
        int moveCount = 0;

        @Override
        public void moveTo(TrackingCommand cmd) {
            this.lastCommand = cmd;
            moveCount++;
        }

        @Override
        public void stop() {}
    }

    /**
     * 测试6：算法一致性验证
     * 思路：
     *   - SmoothTrackingController 设置 maxPredictionCount=0，关闭预测，
     *     使每个雷达点仅触发一次"立即响应"（onRadarData 内部一次 moveTo）
     *   - 同样的 BLH 输入直接调用 RadarOpticTrackerV2.compute()
     *   - 对比两者的 TrackingCommand（除 timestamp 外的所有字段），
     *     验证 SmoothTrackingController/SmoothTracker 与 RadarOpticTrackerV2
     *     使用的核心几何/补偿/距离/宽高算法完全一致
     */
    private static void testAlgorithmConsistencyWithRadarOpticTrackerV2() {
        System.out.println("\n【测试6】算法一致性——与 RadarOpticTrackerV2 对比（不预测）");
        System.out.println("-".repeat(50));

        RadarOpticTrackerV2.Config config = defaultConfig();
        RadarOpticTrackerV2 trackerV2 = new RadarOpticTrackerV2(config);

        CapturingServo servo = new CapturingServo();
        // maxPredictionCount=0：关闭预测，每个雷达点只产生一次立即响应的命令
        SmoothTrackingController ctrl = new SmoothTrackingController(config, servo, 0, 100);

        double[][] points = {
            {39.900, 116.400, 100},
            {39.905, 116.405, 150},
            {39.910, 116.410, 200},
            {39.915, 116.415, 250},
            {39.920, 116.420, 300}
        };
        Double[] areas = { null, 2.5, null, 10.0, 5.0 };

        boolean allPassed = true;
        int mismatchCount = 0;

        for (int i = 0; i < points.length; i++) {
            double b = points[i][0];
            double l = points[i][1];
            double h = points[i][2];
            Double area = areas[i];
            long timestamp = 1000L + i * 1000L;

            // SmoothTrackingController 路径
            ctrl.onRadarData(timestamp, b, l, h, area);
            TrackingCommand ctrlCmd = servo.lastCommand;

            // 第1个点不联动，验证 controller 未发出命令
            if (i == 0) {
                if (ctrlCmd != null) {
                    allPassed = false;
                    mismatchCount++;
                    System.out.printf("点%d 预期不联动但收到命令%n", i);
                }
                continue;
            }

            // RadarOpticTrackerV2 路径（直接计算同一 BLH 点）
            TrackingCommand v2Cmd = trackerV2.compute(b, l, h, null, area);

            if (!commandsEqual(ctrlCmd, v2Cmd, 1e-9)) {
                allPassed = false;
                mismatchCount++;
                System.out.printf("点%d 不一致 (B=%.4f, L=%.4f, H=%.1f, area=%s):%n",
                        i, b, l, h, area);
                System.out.printf("  controller: azCmd=%.9f, elCmd=%.9f, range=%.6f, w=%.6f, h=%.6f%n",
                        ctrlCmd.azCmd(), ctrlCmd.elCmd(), ctrlCmd.opticalToTargetRange(),
                        ctrlCmd.targetWidth(), ctrlCmd.targetHeight());
                System.out.printf("  trackerV2 : azCmd=%.9f, elCmd=%.9f, range=%.6f, w=%.6f, h=%.6f%n",
                        v2Cmd.azCmd(), v2Cmd.elCmd(), v2Cmd.opticalToTargetRange(),
                        v2Cmd.targetWidth(), v2Cmd.targetHeight());
            }
        }

        // 同时验证：onRadarData 在 maxPredictionCount=0 下，除首点外每点恰好触发 1 次 moveTo
        boolean countOk = servo.moveCount == points.length - 1;

        System.out.printf("对比点数: %d，不一致数: %d，moveTo 次数: %d（期望 %d）%n",
                points.length, mismatchCount, servo.moveCount, points.length);
        System.out.println((allPassed && countOk) ? "✓ 通过" : "✗ 失败");
    }

    /**
     * 比较两个 TrackingCommand 的所有数值字段（忽略 timestamp）
     */
    private static boolean commandsEqual(TrackingCommand a, TrackingCommand b, double eps) {
        if (a == null || b == null) return false;
        return Math.abs(a.azCmd() - b.azCmd()) < eps
            && Math.abs(a.elCmd() - b.elCmd()) < eps
            && Math.abs(a.azGeo() - b.azGeo()) < eps
            && Math.abs(a.elGeo() - b.elGeo()) < eps
            && Math.abs(a.dAz0() - b.dAz0()) < eps
            && Math.abs(a.dEl0() - b.dEl0()) < eps
            && Math.abs(a.dAzRadar() - b.dAzRadar()) < eps
            && Math.abs(a.dElRadar() - b.dElRadar()) < eps
            && Math.abs(a.opticalToTargetRange() - b.opticalToTargetRange()) < eps
            && Math.abs(a.targetWidth() - b.targetWidth()) < eps
            && Math.abs(a.targetHeight() - b.targetHeight()) < eps;
    }
}
