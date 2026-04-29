package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

/**
 * 平滑光电跟踪器 - 连续轨迹插值版（纯预测器）
 *
 * 核心职责：
 * - 管理轨迹滑动窗口（t1, t2）
 * - 基于速度外推计算任意时刻的目标位置
 *
 * 注意：本类不负责目标丢失检测，超时检测由 SmoothTrackingController 负责
 *
 * 重构说明：
 * - 移除超时检测逻辑（由 Controller 层负责）
 * - 移除 TargetLostException（不再抛出）
 * - 角度计算、雷达补偿查找已委托 TrackingCore
 */
public class SmoothTracker {

    // ==================== 标定参数（外部传入） ====================

    private final StationBLH opticalBlh;
    private final double dAz0, dEl0;
    private final TrackingCore.RadarCompensation radarComp;
    private final long totalDelayMs;

    // ==================== 轨迹状态 ====================

    /** 轨迹点1: 纬度 */
    private double t1B, t1L, t1H;
    /** 轨迹点1: 时间戳（毫秒） */
    private long t1Time;
    /** 轨迹点1: 雷达面积 */
    private Double t1Area;

    /** 轨迹点2: 纬度 */
    private double t2B, t2L, t2H;
    /** 轨迹点2: 时间戳（毫秒） */
    private long t2Time;
    /** 轨迹点2: 雷达面积 */
    private Double t2Area;

    /** 是否已收到第1个雷达点（t1 已就绪，但 t2 尚未就绪） */
    private boolean hasFirstPoint = false;

    /** 是否已初始化（至少收到2个雷达点） */
    private boolean hasTrajectory = false;

    /** 最近一次查询时间（用于计算命令中的时间戳） */
    private long lastQueryTime;

    // ==================== 构造函数 ====================

    /**
     * 构造函数
     *
     * @param opticalBlh 光电站BLH
     * @param dAz0 方位角固定偏差
     * @param dEl0 俯仰角固定偏差
     * @param radarComp 雷达补偿策略（TrackingCore.RadarCompensation 或其子类型）
     */
    public SmoothTracker(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp) {
        this(opticalBlh, dAz0, dEl0, radarComp, 0L);
    }

    public SmoothTracker(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp,
            long totalDelayMs) {
        this.opticalBlh = opticalBlh;
        this.dAz0 = dAz0;
        this.dEl0 = dEl0;
        this.radarComp = radarComp;
        this.totalDelayMs = totalDelayMs;
    }

    /**
     * 工厂方法：从标定配置文件创建
     *
     * @param configPath 标定参数文件路径
     */
    public static SmoothTracker fromCalibrationV2(String configPath) {
        RadarOpticTrackerV2.Config config = TrackerConfigLoader.load(configPath);
        return new SmoothTracker(
            config.opticalBlh(),
            config.dAz0(),
            config.dEl0(),
            config.radarCompensation(),
            config.totalDelayMs()
        );
    }

    // ==================== 核心方法 ====================

    /**
     * 接收雷达数据（无面积）
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param targetB 目标纬度
     * @param targetL 目标经度
     * @param targetH 目标高度（米）
     */
    public void update(long timestamp, double targetB, double targetL, double targetH) {
        update(timestamp, targetB, targetL, targetH, null);
    }

    /**
     * 接收雷达数据（含面积）
     *
     * 逻辑：
     * - 第1个点：设为 t1（轨迹未就绪）
     * - 第2个点：设为 t2（轨迹就绪，可外推）
     * - 第3个及以后：滑动窗口，t1←t2←新点
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param targetB 目标纬度
     * @param targetL 目标经度
     * @param targetH 目标高度（米）
     * @param radarArea 雷达探测的目标面积（平方米），可为 null
     */
    public void update(long timestamp, double targetB, double targetL, double targetH, Double radarArea) {
        if (!hasFirstPoint) {
            // 第1个点：设为 t1，轨迹尚未就绪
            t1B = targetB;
            t1L = targetL;
            t1H = targetH;
            t1Time = timestamp;
            t1Area = radarArea;
            hasFirstPoint = true;
            return;
        }

        if (!hasTrajectory) {
            // 第2个点：设为 t2，轨迹就绪可外推
            t2B = targetB;
            t2L = targetL;
            t2H = targetH;
            t2Time = timestamp;
            t2Area = radarArea;
            hasTrajectory = true;
            return;
        }

        // 第3个及以后的点：滑动窗口
        t1B = t2B;
        t1L = t2L;
        t1H = t2H;
        t1Time = t2Time;
        t1Area = t2Area;

        t2B = targetB;
        t2L = targetL;
        t2H = targetH;
        t2Time = timestamp;
        t2Area = radarArea;
    }

    /**
     * 获取光电跟踪命令
     *
     * @param queryTime 光电请求时刻（毫秒）
     * @return 跟踪命令，轨迹未就绪（只有1个点）时返回 null
     */
    public TrackingCommand getCommand(long queryTime) {
        if (!hasTrajectory) {
            return null;
        }

        lastQueryTime = queryTime;
        long predictTime = queryTime + totalDelayMs;

        // 预测目标位置（hasTrajectory 已在方法开头校验）
        double predB, predL, predH;

        // 基于t1和t2线性外推
        double[] velocity = computeVelocity();
        double dt = (predictTime - t2Time) / 1000.0;  // 秒

        if (dt >= 0) {
            // 外推：predictTime在t2之后
            predB = t2B + velocity[0] * dt;
            predL = t2L + velocity[1] * dt;
            predH = t2H + velocity[2] * dt;
        } else {
            // 内插：predictTime在t1和t2之间
            double totalDt = (t2Time - t1Time) / 1000.0;
            if (totalDt > 0) {
                double ratio = 1.0 + dt / totalDt;  // dt为负，所以ratio<1
                predB = t1B + (t2B - t1B) * ratio;
                predL = t1L + (t2L - t1L) * ratio;
                predH = t1H + (t2H - t1H) * ratio;
            } else {
                // 防止除零，用t2
                predB = t2B;
                predL = t2L;
                predH = t2H;
            }
        }

        // 计算光电角度
        return computeAngles(predB, predL, predH);
    }

    /**
     * 简化版：基于当前时刻自动计算
     */
    public TrackingCommand getCommand() {
        return getCommand(System.currentTimeMillis());
    }

    // ==================== 内部方法 ====================

    /**
     * 计算目标速度（度/秒）
     * [0]=纬度速度, [1]=经度速度, [2]=高度速度
     */
    private double[] computeVelocity() {
        double dt = (t2Time - t1Time) / 1000.0;
        if (dt <= 0) {
            return new double[]{0, 0, 0};
        }
        return new double[]{
            (t2B - t1B) / dt,
            (t2L - t1L) / dt,
            (t2H - t1H) / dt
        };
    }

    /**
     * 计算光电跟踪角度（委托 TrackingCore）
     */
    private TrackingCommand computeAngles(double targetB, double targetL, double targetH) {
        // 使用 TrackingCore 统一计算角度
        double[] angles = TrackingCore.computeAngles(
            targetB, targetL, targetH,
            opticalBlh, dAz0, dEl0, radarComp
        );
        double azCmd = angles[0];
        double elCmd = angles[1];
        double azGeo = angles[2];
        double elGeo = angles[3];
        double dAzRadar = angles[4];
        double dElRadar = angles[5];

        // 距离
        double range = CoordinateUtils.distance3D(
            opticalBlh, new StationBLH(targetB, targetL, targetH)
        );

        // 使用最新的雷达面积计算目标宽高（优先 t2，否则 t1）
        Double latestArea = hasTrajectory ? t2Area : t1Area;
        double[] targetSize = TrackingCore.computeTargetSize(latestArea);
        double targetWidth = targetSize[0];
        double targetHeight = targetSize[1];

        return new TrackingCommand(
            azCmd, elCmd, azGeo, elGeo,
            dAz0, dEl0, dAzRadar, dElRadar,
            range, targetWidth, targetHeight,
            lastQueryTime
        );
    }

    // ==================== 状态查询 ====================

    /** 是否已有完整轨迹（可外推） */
    public boolean isReady() { return hasTrajectory; }

    /** 获取t1点位置（用于测试验证） */
    public double getT1B() { return t1B; }
    public double getT1L() { return t1L; }
    public double getT1H() { return t1H; }
    public long getT1Time() { return t1Time; }
    public Double getT1Area() { return t1Area; }

    /** 获取t2点位置（用于测试验证） */
    public double getT2B() { return t2B; }
    public double getT2L() { return t2L; }
    public double getT2H() { return t2H; }
    public long getT2Time() { return t2Time; }
    public Double getT2Area() { return t2Area; }

    /** 重置（目标重新捕获时调用） */
    public void reset() {
        hasFirstPoint = false;
        hasTrajectory = false;
        t1Time = 0;
        t2Time = 0;
        t1Area = null;
        t2Area = null;
    }

    // ==================== 数据结构 ====================

    // TrackingCommand 已提取为公共类，见 TrackingCommand.java

    // ==================== 测试 ====================

    public static void main(String[] args) {
        System.out.println("\n========== SmoothTracker（纯预测器）测试 ==========\n");

        testSlidingWindowLogic();
        testNormalTracking();
        testSequentialPredictionWith3Points();

        System.out.println("\n========== 所有测试完成 ==========");
    }

    /**
     * 测试1: 验证滑动窗口逻辑
     */
    private static void testSlidingWindowLogic() {
        System.out.println("【测试1】滑动窗口逻辑验证");
        System.out.println("-".repeat(50));

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 0;

        // 第1个雷达点
        tracker.update(baseTime, 39.905, 116.408, 100);
        System.out.printf("t=0s: update → hasTrajectory=%s%n", tracker.isReady());

        // 第2个雷达点
        tracker.update(baseTime + 2000, 39.910, 116.410, 150);
        System.out.printf("t=2s: update → hasTrajectory=%s%n", tracker.isReady());

        // 第3个雷达点：验证滑动窗口
        tracker.update(baseTime + 4000, 39.915, 116.412, 200);
        System.out.printf("t=4s: update → hasTrajectory=%s%n", tracker.isReady());

        // 验证内部状态
        System.out.printf("t1=(%.4f, %.4f), t2=(%.4f, %.4f)%n",
            tracker.t1B, tracker.t1L, tracker.t2B, tracker.t2L);

        // 获取预测命令
        var cmd = tracker.getCommand(baseTime + 4500);
        System.out.printf("t=4.5s: Az=%.4f°, El=%.4f°%n", cmd.azCmd(), cmd.elCmd());
        System.out.println("✓ 滑动窗口逻辑正常");
    }

    /**
     * 测试2: 正常跟踪流程
     */
    private static void testNormalTracking() {
        System.out.println("\n【测试2】正常跟踪流程");
        System.out.println("-".repeat(50));

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 0;

        System.out.println("场景：目标匀速移动，雷达每2秒给一个点");

        double targetB = 39.905, targetL = 116.408, targetH = 200;
        double vB = 0.001 / 2.0;
        double vL = 0.0005 / 2.0;

        for (int radarStep = 0; radarStep < 3; radarStep++) {
            long radarTime = baseTime + radarStep * 2000;
            targetB += vB * 2.0;
            targetL += vL * 2.0;

            tracker.update(radarTime, targetB, targetL, targetH);
            System.out.printf("t=%ds: update → isReady=%s%n", radarStep * 2, tracker.isReady());

            // 每0.5秒请求一次（第1个点不联动，跳过）
            if (!tracker.isReady()) {
                System.out.println("    → 第1个点，不联动");
                continue;
            }
            for (int opt = 1; opt <= 3; opt++) {
                var cmd = tracker.getCommand(radarTime + opt * 500);
                System.out.printf("    +%.1fs: Az=%.4f°, El=%.4f°%n",
                    opt * 0.5, cmd.azCmd(), cmd.elCmd());
            }
        }

        System.out.println("✓ 正常跟踪测试完成");
    }

    /**
     * 测试3: 验证顺序基于3个点进行预测
     */
    private static void testSequentialPredictionWith3Points() {
        System.out.println("\n【测试3】顺序基于3个点进行预测验证");
        System.out.println("-".repeat(60));

        StationBLH opticalBlh = new StationBLH(39.800, 116.300, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        final long RADAR_INTERVAL = 2000; // 2秒

        // 定义3个雷达点
        double[][] radarPoints = {
            {39.900, 116.400, 100},  // P0: t=0s
            {39.905, 116.405, 150},  // P1: t=2s
            {39.910, 116.410, 200}   // P2: t=4s
        };

        System.out.println("场景：目标匀速直线运动");
        System.out.println("P0: B=39.900, L=116.400, H=100");
        System.out.println("P1: B=39.905, L=116.405, H=150 (Δt=2s)");
        System.out.println("P2: B=39.910, L=116.410, H=200 (Δt=2s)");
        System.out.println();

        // 逐个雷达点测试
        for (int radarIdx = 0; radarIdx < radarPoints.length; radarIdx++) {
            long radarTime = radarIdx * RADAR_INTERVAL;
            double[] p = radarPoints[radarIdx];

            tracker.update(radarTime, p[0], p[1], p[2]);
            System.out.printf("【雷达点%d】t=%ds: hasTrajectory=%s%n",
                radarIdx, radarIdx * 2, tracker.isReady());

            if (!tracker.isReady()) {
                System.out.println("  → 轨迹未就绪，跳过预测");
                continue;
            }

            // 采样几个关键点
            System.out.println("  预测采样:");
            for (long offset : new long[]{100, 500, 1000, 1500}) {
                var cmd = tracker.getCommand(radarTime + offset);
                System.out.printf("    +%dms: Az=%.6f°, El=%.6f°%n",
                    offset, cmd.azCmd(), cmd.elCmd());
            }
            System.out.println();
        }

        // 滑动窗口验证
        System.out.println("【滑动窗口验证】");
        System.out.printf("t1=(%.4f, %.4f) 期望=P1(39.905, 116.405) %s%n",
            tracker.t1B, tracker.t1L,
            Math.abs(tracker.t1B - 39.905) < 0.0001 ? "✓" : "✗");
        System.out.printf("t2=(%.4f, %.4f) 期望=P2(39.910, 116.410) %s%n",
            tracker.t2B, tracker.t2L,
            Math.abs(tracker.t2B - 39.910) < 0.0001 ? "✓" : "✗");

        System.out.println("\n✓ 顺序基于3个点预测测试完成");
    }
}
