package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * 平滑跟踪控制器调用器（第三方接入层）
 *
 * 提供全局默认实例，同时支持多实例场景。
 *
 * 用法：
 * <pre>
 * SmoothTrackingControllerInvoker invoker = SmoothTrackingControllerInvoker.getDefault();
 * invoker.register("ctrl-001", "calibration_v2.json", servo, 10, 20);
 * invoker.get("ctrl-001").ifPresent(c -> c.onRadarData(System.currentTimeMillis(), b, l, h));
 * </pre>
 */
public class SmoothTrackingControllerInvoker {

    // ==================== 全局默认实例 ====================

    private static final SmoothTrackingControllerInvoker DEFAULT = new SmoothTrackingControllerInvoker();

    /**
     * 获取全局默认实例
     */
    public static SmoothTrackingControllerInvoker getDefault() {
        return DEFAULT;
    }

    // ==================== 实例成员 ====================

    /** key → 平滑跟踪控制器实例 */
    private final Map<String, SmoothTrackingController> controllers = new ConcurrentHashMap<>();

    // ==================== 实例方法：注册/管理 ====================

    /**
     * 注册控制器（从配置文件路径构造，幂等：已存在则覆盖）
     *
     * @param key                实例标识（如 "ctrl-001"）
     * @param configPath         标定参数文件路径
     * @param servo              伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次
     * @param tickIntervalMs     光电控制周期（ms）
     * @return 该 key 对应的控制器实例
     */
    public SmoothTrackingController register(String key, String configPath,
                                             SmoothTrackingController.ServoController servo,
                                             int maxPredictionCount, long tickIntervalMs) {
        SmoothTrackingController ctrl = new SmoothTrackingController(
                configPath, servo, maxPredictionCount, tickIntervalMs);
        controllers.put(key, ctrl);
        System.out.println("[SmoothTrackingControllerInvoker] 注册实例: " + key);
        return ctrl;
    }

    /**
     * 注册控制器（从配置对象构造，幂等：已存在则覆盖）
     *
     * @param key                实例标识
     * @param config             标定配置
     * @param servo              伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次
     * @param tickIntervalMs     光电控制周期（ms）
     * @return 该 key 对应的控制器实例
     */
    public SmoothTrackingController register(String key, RadarOpticTrackerV2.Config config,
                                             SmoothTrackingController.ServoController servo,
                                             int maxPredictionCount, long tickIntervalMs) {
        SmoothTrackingController ctrl = new SmoothTrackingController(
                config, servo, maxPredictionCount, tickIntervalMs);
        controllers.put(key, ctrl);
        System.out.println("[SmoothTrackingControllerInvoker] 注册实例: " + key);
        return ctrl;
    }

    /**
     * 注册控制器（从 StationBLH 等参数构造）
     *
     * @param key                实例标识
     * @param opticalBlh         光电站BLH
     * @param dAz0               方位角固定偏差
     * @param dEl0               俯仰角固定偏差
     * @param radarComp          雷达补偿策略
     * @param servo              伺服控制器回调
     * @param maxPredictionCount 每次新点后最多预测N次
     * @param tickIntervalMs     光电控制周期（ms）
     * @param totalDelayMs       雷达到光电的总时延（ms）
     * @return 该 key 对应的控制器实例
     */
    public SmoothTrackingController register(String key,
                                             StationBLH opticalBlh,
                                             double dAz0, double dEl0,
                                             TrackingCore.RadarCompensation radarComp,
                                             SmoothTrackingController.ServoController servo,
                                             int maxPredictionCount, long tickIntervalMs, long totalDelayMs) {
        SmoothTrackingController ctrl = new SmoothTrackingController(
                opticalBlh, dAz0, dEl0, radarComp, servo,
                maxPredictionCount, tickIntervalMs, totalDelayMs);
        controllers.put(key, ctrl);
        System.out.println("[SmoothTrackingControllerInvoker] 注册实例: " + key);
        return ctrl;
    }

    /**
     * 获取控制器
     *
     * @param key 实例标识
     * @return 控制器实例，若不存在则返回空 Optional
     */
    public Optional<SmoothTrackingController> get(String key) {
        return Optional.ofNullable(controllers.get(key));
    }

    /**
     * 检查控制器是否已注册
     */
    public boolean contains(String key) {
        return controllers.containsKey(key);
    }

    /**
     * 移除控制器
     *
     * @param key 实例标识
     * @return 被移除的实例，若不存在则返回 null
     */
    public SmoothTrackingController remove(String key) {
        return controllers.remove(key);
    }

    /**
     * 列出所有已注册的 key
     */
    public Set<String> listKeys() {
        return Set.copyOf(controllers.keySet());
    }

    /**
     * 清空所有控制器实例
     */
    public void clear() {
        controllers.clear();
    }
}
