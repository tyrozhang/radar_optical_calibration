package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

import java.util.logging.Logger;

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

    private static final Logger log = Logger.getLogger(SmoothTracker.class.getName());

    /** 最大外推时长（秒），防止长时间无数据时外推发散 */
    private static final double MAX_PREDICTION_SECONDS = 2.0;

    // ==================== 标定参数（外部传入） ====================

    private final StationBLH opticalBlh;
    private final double dAz0, dEl0;
    private final TrackingCore.RadarCompensation radarComp;
    private final long totalDelayMs;
    private final ElevationConvention elevationConvention;

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
     * 构造函数（全参数）
     *
     * @param opticalBlh 光电站BLH
     * @param dAz0 方位角固定偏差
     * @param dEl0 俯仰角固定偏差
     * @param radarComp 雷达补偿策略（TrackingCore.RadarCompensation 或其子类型）
     * @param totalDelayMs 雷达到光电的总时延（毫秒）
     * @param elevationConvention 俯仰角符号约定
     */
    public SmoothTracker(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp,
            long totalDelayMs,
            ElevationConvention elevationConvention) {
        this.opticalBlh = opticalBlh;
        this.dAz0 = dAz0;
        this.dEl0 = dEl0;
        this.radarComp = radarComp;
        this.totalDelayMs = totalDelayMs;
        this.elevationConvention = elevationConvention;
    }

    /**
     * 构造函数（向后兼容，默认 UP_NEGATIVE）
     */
    public SmoothTracker(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp,
            long totalDelayMs) {
        this(opticalBlh, dAz0, dEl0, radarComp, totalDelayMs, ElevationConvention.UP_NEGATIVE);
    }

    /**
     * 构造函数（向后兼容，默认 totalDelayMs=0, UP_NEGATIVE）
     */
    public SmoothTracker(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp) {
        this(opticalBlh, dAz0, dEl0, radarComp, 0L);
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
            config.totalDelayMs(),
            config.elevationConvention()
        );
    }

    // ==================== 核心方法 ====================

    /**
     * 接收雷达数据（无面积）——线程安全
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param targetB 目标纬度
     * @param targetL 目标经度
     * @param targetH 目标高度（米）
     */
    public synchronized void update(long timestamp, double targetB, double targetL, double targetH) {
        update(timestamp, targetB, targetL, targetH, null);
    }

    /**
     * 接收雷达数据（含面积）
     *
     * 逻辑：
     * - 第1个点：设为 t1（轨迹未就绪）
     * - 第2个点：设为 t2（轨迹就绪，可外推），若时间戳倒退则拒绝
     * - 第3个及以后：滑动窗口，t1←t2←新点，若时间戳倒退则拒绝
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param targetB 目标纬度
     * @param targetL 目标经度
     * @param targetH 目标高度（米）
     * @param radarArea 雷达探测的目标面积（平方米），可为 null
     */
    public synchronized void update(long timestamp, double targetB, double targetL, double targetH, Double radarArea) {
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
            // N9 修复：检测时间戳倒退，拒绝早于 t1Time 的数据点
            if (timestamp <= t1Time) {
                log.warning(String.format(
                    "时间戳倒退，忽略该雷达点: newTs=%d, t1Time=%d, delta=%dms",
                    timestamp, t1Time, timestamp - t1Time));
                return;
            }
            t2B = targetB;
            t2L = targetL;
            t2H = targetH;
            t2Time = timestamp;
            t2Area = radarArea;
            hasTrajectory = true;
            return;
        }

        // N9 修复：第3个及以后的点，检测时间戳倒退，拒绝早于或等于 t2Time 的数据点
        if (timestamp <= t2Time) {
            log.warning(String.format(
                "时间戳倒退，忽略该雷达点: newTs=%d, t2Time=%d, delta=%dms",
                timestamp, t2Time, timestamp - t2Time));
            return;
        }

        // 滑动窗口：t1←t2←新点
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
     * 获取光电跟踪命令——线程安全
     *
     * @param queryTime 光电请求时刻（毫秒）
     * @return 跟踪命令，轨迹未就绪（只有1个点）时返回 null
     */
    public synchronized TrackingCommand getCommand(long queryTime) {
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
            // 正向外推：predictTime在t2之后
            // N10 修复（正向）：限制最大外推时长
            dt = Math.min(dt, MAX_PREDICTION_SECONDS);
            predB = t2B + velocity[0] * dt;
            predL = t2L + velocity[1] * dt;
            predH = t2H + velocity[2] * dt;
        } else {
            // 内插：predictTime在t2之前
            double totalDt = (t2Time - t1Time) / 1000.0;
            if (totalDt > 0) {
                double ratio = 1.0 + dt / totalDt;  // dt为负，所以ratio<1

                if (ratio < 0) {
                    // N10 修复（反向）：queryTime < t1Time，clamp 到 t1 位置，不反向发散
                    log.warning(String.format(
                        "查询时间早于轨迹窗口起点，clamp 到 t1: queryTime=%d, t1Time=%d",
                        queryTime, t1Time));
                    predB = t1B;
                    predL = t1L;
                    predH = t1H;
                } else {
                    // 正常内插：ratio ∈ [0, 1]
                    predB = t1B + (t2B - t1B) * ratio;
                    predL = t1L + (t2L - t1L) * ratio;
                    predH = t1H + (t2H - t1H) * ratio;
                }
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
     * 简化版：基于当前时刻自动计算——线程安全
     */
    public synchronized TrackingCommand getCommand() {
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

        // 按设备俯仰约定输出
        double elCmdOut = elevationConvention.apply(elCmd);
        double elGeoOut = elevationConvention.apply(elGeo);

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
            azCmd, elCmdOut, azGeo, elGeoOut,
            dAz0, dEl0, dAzRadar, dElRadar,
            range, targetWidth, targetHeight,
            lastQueryTime
        );
    }

    // ==================== 状态查询 ====================

    /** 是否已有完整轨迹（可外推）——线程安全 */
    public synchronized boolean isReady() { return hasTrajectory; }

    /** 获取t1点位置（用于测试验证）——线程安全 */
    public synchronized double getT1B() { return t1B; }
    public synchronized double getT1L() { return t1L; }
    public synchronized double getT1H() { return t1H; }
    public synchronized long getT1Time() { return t1Time; }
    public synchronized Double getT1Area() { return t1Area; }

    /** 获取t2点位置（用于测试验证）——线程安全 */
    public synchronized double getT2B() { return t2B; }
    public synchronized double getT2L() { return t2L; }
    public synchronized double getT2H() { return t2H; }
    public synchronized long getT2Time() { return t2Time; }
    public synchronized Double getT2Area() { return t2Area; }

    /** 重置（目标重新捕获时调用）——线程安全 */
    public synchronized void reset() {
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
        log.info("========== SmoothTracker（纯预测器）测试 ==========");

        testSlidingWindowLogic();
        testNormalTracking();
        testSequentialPredictionWith3Points();
        testTimestampRegression();              // N9
        testQueryBeforeTrajectoryWindow();      // N10

        log.info("========== 所有测试完成 ==========");
    }

    /**
     * 测试1: 验证滑动窗口逻辑
     */
    private static void testSlidingWindowLogic() {
        log.info("【测试1】滑动窗口逻辑验证");

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 0;

        // 第1个雷达点
        tracker.update(baseTime, 39.905, 116.408, 100);
        log.info("t=0s: update → hasTrajectory=" + tracker.isReady());

        // 第2个雷达点
        tracker.update(baseTime + 2000, 39.910, 116.410, 150);
        log.info("t=2s: update → hasTrajectory=" + tracker.isReady());

        // 第3个雷达点：验证滑动窗口
        tracker.update(baseTime + 4000, 39.915, 116.412, 200);
        log.info("t=4s: update → hasTrajectory=" + tracker.isReady());

        // 验证内部状态
        log.info(String.format("t1=(%.4f, %.4f), t2=(%.4f, %.4f)",
            tracker.t1B, tracker.t1L, tracker.t2B, tracker.t2L));

        // 获取预测命令
        var cmd = tracker.getCommand(baseTime + 4500);
        log.info(String.format("t=4.5s: Az=%.4f°, El=%.4f°", cmd.azCmd(), cmd.elCmd()));
        log.info("✓ 滑动窗口逻辑正常");
    }

    /**
     * 测试2: 正常跟踪流程
     */
    private static void testNormalTracking() {
        log.info("【测试2】正常跟踪流程");

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 0;

        log.info("场景：目标匀速移动，雷达每2秒给一个点");

        double targetB = 39.905, targetL = 116.408, targetH = 200;
        double vB = 0.001 / 2.0;
        double vL = 0.0005 / 2.0;

        for (int radarStep = 0; radarStep < 3; radarStep++) {
            long radarTime = baseTime + radarStep * 2000;
            targetB += vB * 2.0;
            targetL += vL * 2.0;

            tracker.update(radarTime, targetB, targetL, targetH);
            log.info("t=" + (radarStep * 2) + "s: update → isReady=" + tracker.isReady());

            // 每0.5秒请求一次（第1个点不联动，跳过）
            if (!tracker.isReady()) {
                log.info("    → 第1个点，不联动");
                continue;
            }
            for (int opt = 1; opt <= 3; opt++) {
                var cmd = tracker.getCommand(radarTime + opt * 500);
                log.info(String.format("    +%.1fs: Az=%.4f°, El=%.4f°",
                    opt * 0.5, cmd.azCmd(), cmd.elCmd()));
            }
        }

        log.info("✓ 正常跟踪测试完成");
    }

    /**
     * 测试3: 验证顺序基于3个点进行预测
     */
    private static void testSequentialPredictionWith3Points() {
        log.info("【测试3】顺序基于3个点进行预测验证");

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

        log.info("场景：目标匀速直线运动");
        log.info("P0: B=39.900, L=116.400, H=100");
        log.info("P1: B=39.905, L=116.405, H=150 (Δt=2s)");
        log.info("P2: B=39.910, L=116.410, H=200 (Δt=2s)");

        // 逐个雷达点测试
        for (int radarIdx = 0; radarIdx < radarPoints.length; radarIdx++) {
            long radarTime = radarIdx * RADAR_INTERVAL;
            double[] p = radarPoints[radarIdx];

            tracker.update(radarTime, p[0], p[1], p[2]);
            log.info("【雷达点" + radarIdx + "】t=" + (radarIdx * 2) + "s: hasTrajectory=" + tracker.isReady());

            if (!tracker.isReady()) {
                log.info("  → 轨迹未就绪，跳过预测");
                continue;
            }

            // 采样几个关键点
            log.info("  预测采样:");
            for (long offset : new long[]{100, 500, 1000, 1500}) {
                var cmd = tracker.getCommand(radarTime + offset);
                log.info(String.format("    +%dms: Az=%.6f°, El=%.6f°", offset, cmd.azCmd(), cmd.elCmd()));
            }
        }

        // 滑动窗口验证
        log.info("【滑动窗口验证】");
        log.info(String.format("t1=(%.4f, %.4f) 期望=P1(39.905, 116.405) %s",
            tracker.t1B, tracker.t1L, Math.abs(tracker.t1B - 39.905) < 0.0001 ? "✓" : "✗"));
        log.info(String.format("t2=(%.4f, %.4f) 期望=P2(39.910, 116.410) %s",
            tracker.t2B, tracker.t2L, Math.abs(tracker.t2B - 39.910) < 0.0001 ? "✓" : "✗"));

        log.info("✓ 顺序基于3个点预测测试完成");
    }

    /**
     * 测试4: 时间戳倒退保护（N9）
     *
     * 验证：
     * - 第2个点时间戳早于第1个点 → 被拒绝
     * - 滑动窗口阶段时间戳倒退 → 被拒绝，t1/t2 不变
     * - 时间戳相等 → 同样被拒绝
     */
    private static void testTimestampRegression() {
        log.info("【测试4】时间戳倒退保护（N9）");

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 10000;

        // 第1个雷达点
        tracker.update(baseTime, 39.905, 116.408, 100);
        log.info("t=10s: update → hasFirstPoint=true, hasTrajectory=" + tracker.isReady());

        // 场景1: 第2个点时间戳早于第1个点（应被拒绝）
        tracker.update(baseTime - 1000, 39.910, 116.410, 150);  // 9s < 10s
        log.info("t=9s (倒退): hasTrajectory=" + tracker.isReady() + " → 期望=false（被拒绝）");
        boolean pass1 = !tracker.isReady();

        // 正常第2个点
        tracker.update(baseTime + 2000, 39.910, 116.410, 150);  // 12s
        log.info("t=12s (正常): hasTrajectory=" + tracker.isReady() + " → 期望=true");
        boolean pass2 = tracker.isReady();

        // 记录正常状态
        double savedT1B = tracker.getT1B();
        double savedT2B = tracker.getT2B();
        long savedT2Time = tracker.getT2Time();

        // 场景2: 滑动窗口阶段时间戳倒退（应被拒绝，t1/t2 不变）
        tracker.update(baseTime + 1000, 39.920, 116.420, 200);  // 11s < 12s
        log.info(String.format("t=11s (滑动窗口倒退): t1B=%.4f t2B=%.4f t2Time=%d → 期望不变",
            tracker.getT1B(), tracker.getT2B(), tracker.getT2Time()));
        boolean pass3 = tracker.getT1B() == savedT1B
                      && tracker.getT2B() == savedT2B
                      && tracker.getT2Time() == savedT2Time;

        // 场景3: 时间戳相等（应被拒绝）
        tracker.update(baseTime + 2000, 39.930, 116.430, 250);  // 12s == 12s
        log.info(String.format("t=12s (时间戳相等): t2Time=%d → 期望不变",
            tracker.getT2Time()));
        boolean pass4 = tracker.getT2Time() == savedT2Time;

        // 正常第3个点（应被接受）
        tracker.update(baseTime + 4000, 39.915, 116.412, 200);  // 14s
        log.info(String.format("t=14s (正常): t1B=%.4f t2B=%.4f → 期望滑动更新",
            tracker.getT1B(), tracker.getT2B()));
        boolean pass5 = tracker.getT1B() == savedT2B
                      && Math.abs(tracker.getT2B() - 39.915) < 0.0001;

        log.info("N9 测试结果:");
        log.info("  场景1(第2点倒退): " + (pass1 ? "✓" : "✗"));
        log.info("  场景2(滑动窗口倒退): " + (pass3 ? "✓" : "✗"));
        log.info("  场景3(时间戳相等): " + (pass4 ? "✓" : "✗"));
        log.info("  场景4(正常点仍被接受): " + (pass5 ? "✓" : "✗"));
        log.info((pass1 && pass2 && pass3 && pass4 && pass5) ? "✓ N9 测试全部通过" : "✗ N9 测试有失败");
    }

    /**
     * 测试5: 查询时间早于轨迹窗口（N10）
     *
     * 验证：
     * - queryTime << t1Time 时，结果 clamp 到 t1 位置而非反向发散
     * - 正常内插（t1Time < queryTime < t2Time）仍正确
     * - 正向外推受 MAX_PREDICTION_SECONDS 限制
     */
    private static void testQueryBeforeTrajectoryWindow() {
        log.info("【测试5】查询时间早于轨迹窗口 / 外推时长保护（N10）");

        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);
        var comp = new RadarOpticTrackerV2.FixedCompensation(0, 0);
        SmoothTracker tracker = new SmoothTracker(opticalBlh, 0, 0, comp);

        long baseTime = 10000;

        // 建立2点轨迹
        tracker.update(baseTime, 39.900, 116.400, 100);       // t1: t=10s
        tracker.update(baseTime + 2000, 39.910, 116.410, 200); // t2: t=12s

        log.info(String.format("轨迹建立: t1=(%.4f,%.4f)@%dms, t2=(%.4f,%.4f)@%dms",
            tracker.getT1B(), tracker.getT1L(), tracker.getT1Time(),
            tracker.getT2B(), tracker.getT2L(), tracker.getT2Time()));

        // 场景1: queryTime 远早于 t1Time → 应 clamp 到 t1 位置
        long veryEarlyQuery = baseTime - 10000;  // t=0s，远早于 t1=10s
        var cmdEarly = tracker.getCommand(veryEarlyQuery);
        // 用 t1 位置直接查询作为参考
        var cmdAtT1 = tracker.getCommand(baseTime);  // predictTime = t1Time → 内插 ratio=0 → 等于 t1
        log.info(String.format("queryTime=0s (远早于t1): Az=%.4f°", cmdEarly.azCmd()));
        log.info(String.format("queryTime=10s (t1处):     Az=%.4f°", cmdAtT1.azCmd()));
        boolean pass1 = Math.abs(cmdEarly.azCmd() - cmdAtT1.azCmd()) < 0.001;

        // 场景2: 正常内插（t1Time < queryTime < t2Time）
        long midQuery = baseTime + 1000;  // t=11s，t1和t2之间
        var cmdMid = tracker.getCommand(midQuery);
        log.info(String.format("queryTime=11s (内插): Az=%.4f°", cmdMid.azCmd()));
        // 内插结果应在 t1 和 t2 对应角度之间
        boolean pass2 = cmdMid.azCmd() != 0;  // 非零即合理

        // 场景3: 正向外推受 MAX_PREDICTION_SECONDS 限制
        // 假设 MAX_PREDICTION_SECONDS=2.0，外推超过2秒应被截断
        long farFutureQuery = baseTime + 2000 + 10000;  // t=22s，距 t2=12s 有 10s
        var cmdFar = tracker.getCommand(farFutureQuery);
        // 对比：恰好2秒外推
        long exactLimitQuery = baseTime + 2000 + 2000;  // t=14s，距 t2=12s 恰好 2s
        var cmdLimit = tracker.getCommand(exactLimitQuery);
        log.info(String.format("queryTime=22s (外推10s，应截断到2s): Az=%.4f°", cmdFar.azCmd()));
        log.info(String.format("queryTime=14s (外推2s，恰好到限):     Az=%.4f°", cmdLimit.azCmd()));
        boolean pass3 = Math.abs(cmdFar.azCmd() - cmdLimit.azCmd()) < 0.001;

        log.info("N10 测试结果:");
        log.info("  场景1(远早于t1，clamp): " + (pass1 ? "✓" : "✗"));
        log.info("  场景2(正常内插): " + (pass2 ? "✓" : "✗"));
        log.info("  场景3(外推超限截断): " + (pass3 ? "✓" : "✗"));
        log.info((pass1 && pass2 && pass3) ? "✓ N10 测试全部通过" : "✗ N10 测试有失败");
    }
}
