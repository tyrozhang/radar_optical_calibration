package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.TrackingCommand;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;

/**
 * SmoothTrackingController - 平滑跟踪控制器
 * 
 * 设计原则：
 * 1. 不依赖"雷达周期假设"——以实际雷达点到达为触发
 * 2. 以预测次数 N 控制平滑阶段长度——不硬编码时间窗口
 * 3. 预测完成后停止发送命令——光电保持最后位置不动
 * 
 * 状态机：
 *   [新雷达点到达] ──→ 立即执行原始位置，重置计数
 *         │
 *         ▼
 *   [预测阶段] ──→ 执行 N 次速度外推预测（每次光电 tick 一次）
 *         │
 *         ▼
 *   [等待] ──→ 不发命令，光电保持最后位置
 *         │
 *         ▼
 *   [下一雷达点到达] ──→ 回到第一步
 */
public class SmoothTrackingController {

    private final SmoothTracker tracker;
    private final ServoController servo;

    // 配置
    private final int maxPredictionCount;  // 每次新点后最多预测N次
    private final long predictionInterval; // 光电控制周期(ms)，用于日志
    private final long totalDelayMs;       // 雷达到光电的总时延(ms)

    // 状态
    private int predictionCount = 0;       // 当前周期已预测次数
    private boolean hasFirstPoint = false; // 是否已有首个雷达点
    private volatile boolean running = false;

    public interface ServoController {
        void moveTo(TrackingCommand cmd);
        void stop();
    }

    /**
     * 构造函数
     *
     * @param opticalBlh 光电站BLH
     * @param dAz0 方位角固定偏差
     * @param dEl0 俯仰角固定偏差
     * @param radarComp 雷达补偿策略
     * @param servo 伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次（默认10次）
     * @param predictionIntervalMs 光电控制周期（默认20ms=50Hz）
     * @param totalDelayMs 雷达到光电的总时延（默认0ms）
     */
    public SmoothTrackingController(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp,
            ServoController servo,
            int maxPredictionCount,
            long predictionIntervalMs,
            long totalDelayMs) {
        this.tracker = new SmoothTracker(opticalBlh, dAz0, dEl0, radarComp, totalDelayMs);
        this.servo = servo;
        this.maxPredictionCount = maxPredictionCount;
        this.predictionInterval = predictionIntervalMs;
        this.totalDelayMs = totalDelayMs;
    }

    /**
     * 构造函数（默认参数）
     *
     * @param opticalBlh 光电站BLH
     * @param dAz0 方位角固定偏差
     * @param dEl0 俯仰角固定偏差
     * @param radarComp 雷达补偿策略
     * @param servo 伺服控制器回调
     */
    public SmoothTrackingController(
            StationBLH opticalBlh,
            double dAz0, double dEl0,
            TrackingCore.RadarCompensation radarComp,
            ServoController servo) {
        this(opticalBlh, dAz0, dEl0, radarComp, servo, 10, 20, 0L);
    }

    /**
     * 构造函数（从标定配置构造）
     *
     * @param config 标定配置
     * @param servo 伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次
     * @param predictionIntervalMs 光电控制周期
     */
    public SmoothTrackingController(
            RadarOpticTrackerV2.Config config,
            ServoController servo,
            int maxPredictionCount,
            long predictionIntervalMs) {
        this(config.opticalBlh(), config.dAz0(), config.dEl0(), config.radarCompensation(), servo, maxPredictionCount, predictionIntervalMs, config.totalDelayMs());
    }

    /**
     * 构造函数（从配置文件路径构造）
     *
     * @param configPath 标定配置文件路径
     * @param servo 伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次
     * @param predictionIntervalMs 光电控制周期
     */
    public SmoothTrackingController(
            String configPath,
            ServoController servo,
            int maxPredictionCount,
            long predictionIntervalMs) {
        this(TrackerConfigLoader.load(configPath), servo, maxPredictionCount, predictionIntervalMs);
    }

    /**
     * 构造函数（从配置文件路径构造，默认控制参数）
     *
     * @param configPath 标定配置文件路径
     * @param servo 伺服控制器回调
     */
    public SmoothTrackingController(String configPath, ServoController servo) {
        this(configPath, servo, 10, 20);
    }

    /**
     * 雷达数据到达时调用（每收到一个雷达点调用一次）
     * 
     * 行为：
     * 1. 更新跟踪器内部状态
     * 2. 重置预测计数
     * 3. 立即执行该雷达点的原始位置
     * 
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param b 目标纬度
     * @param l 目标经度
     * @param h 目标高度
     */
    public void onRadarData(long timestamp, double b, double l, double h) {
        onRadarData(timestamp, b, l, h, null);
    }

    /**
     * 雷达数据到达时调用（带目标面积）
     *
     * 行为：
     * 1. 更新跟踪器内部状态（含目标面积）
     * 2. 重置预测计数
     * 3. 立即执行该雷达点的原始位置
     *
     * @param timestamp 雷达数据时间戳（毫秒）
     * @param b 目标纬度
     * @param l 目标经度
     * @param h 目标高度
     * @param radarArea 目标雷达截面积（可为 null）
     */
    public void onRadarData(long timestamp, double b, double l, double h, Double radarArea) {
        // 1. 更新跟踪器
        tracker.update(timestamp, b, l, h, radarArea);

        // 2. 重置预测计数，进入新周期
        predictionCount = 0;
        hasFirstPoint = true;

        // 3. 轨迹就绪后立即执行（第1个点不联动，第2个点开始联动）
        TrackingCommand cmd = tracker.getCommand(timestamp);
        if (cmd != null) {
            servo.moveTo(cmd);
        }
    }

    /**
     * 启动光电控制循环
     * 
     * 以固定周期调用 tick()，周期由 predictionInterval 控制
     */
    public void start() {
        running = true;
        new Thread(() -> {
            while (running) {
                tick();
                sleep(predictionInterval);
            }
        }, "TrackingController").start();
    }

    public void stop() {
        running = false;
    }

    /**
     * 重置控制器状态（切换目标时调用）
     *
     * 清空当前轨迹和预测计数，准备接收新目标的雷达数据。
     * 无需先调用 stop()，可在运行中直接调用。
     */
    public void reset() {
        this.predictionCount = 0;
        this.hasFirstPoint = false;
        this.tracker.reset();
    }

    /**
     * 单次控制周期（仅供 start() 启动的内部循环调用）
     *
     * 行为：
     * - 如果还在等待首个雷达点：不发命令
     * - 如果只有1个点（轨迹未就绪）：不发命令（速度未知，无法外推）
     * - 如果在预测阶段且未达到 N 次：执行速度外推预测
     * - 如果预测次数已达 N 次：停止发送命令，光电保持最后位置
     */
    private void tick() {
        long now = System.currentTimeMillis();

        // 等待首个雷达点
        if (!hasFirstPoint) {
            return;
        }

        // 轨迹未就绪（仅有1个点，无法计算速度做外推）
        if (!tracker.isReady()) {
            return;
        }

        // 预测计数 +1
        predictionCount++;

        if (predictionCount <= maxPredictionCount) {
            // 还在预测阶段：执行速度外推
            TrackingCommand cmd = tracker.getCommand(now);
            servo.moveTo(cmd);
        }
        // 超过 maxPredictionCount 次：什么都不做，光电保持最后位置
    }

    /**
     * 获取当前预测次数（用于测试和调试）
     */
    public int getPredictionCount() {
        return predictionCount;
    }

    /**
     * 跟踪轨迹是否已具备外推能力（至少有 2 个雷达点，速度可计算）
     */
    public boolean isTrackerReady() {
        return tracker.isReady();
    }

    /**
     * 是否在等待首个雷达点
     */
    public boolean isWaitingFirstPoint() {
        return !hasFirstPoint;
    }

    /**
     * 是否在预测阶段（未达到 N 次预测）
     */
    public boolean isInPredictionPhase() {
        return hasFirstPoint && predictionCount <= maxPredictionCount;
    }

    /**
     * 是否已完成预测（等待下一雷达点）
     */
    public boolean isWaitingNextPoint() {
        return hasFirstPoint && predictionCount > maxPredictionCount;
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    /** 获取内部 tracker（用于测试） */
    SmoothTracker getTracker() {
        return tracker;
    }
}
