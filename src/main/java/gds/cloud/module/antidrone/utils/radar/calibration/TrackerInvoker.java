package gds.cloud.module.antidrone.utils.radar.calibration;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * 跟踪程序调用器（第三方接入层）
 *
 * 提供全局默认实例，同时支持多实例场景。
 *
 * 用法：
 * <pre>
 * // 方式1：使用全局默认实例
 * TrackerInvoker invoker = TrackerInvoker.getDefault();
 * invoker.register("tracker-001", "calibration_v2.json");
 * TrackingCommand cmd = invoker.compute("tracker-001", B, L, H, range, area);
 *
 * // 方式2：独立实例
 * TrackerInvoker invoker2 = new TrackerInvoker();
 * invoker2.register("tracker-002", "calibration_other.json");
 * </pre>
 */
public class TrackerInvoker {

    // ==================== 全局默认实例 ====================

    private static final TrackerInvoker DEFAULT = new TrackerInvoker();

    /**
     * 获取全局默认实例
     */
    public static TrackerInvoker getDefault() {
        return DEFAULT;
    }

    // ==================== 实例成员 ====================

    /** key → 跟踪器实例 */
    private final Map<String, RadarOpticTrackerV2> trackers = new ConcurrentHashMap<>();

    // ==================== 实例方法 ====================

    /**
     * 注册跟踪器（幂等：已存在则复用，不会重建）
     *
     * @param key        实例标识（如 "tracker-001"）
     * @param configPath 标定参数文件路径
     * @return 该 key 对应的跟踪器实例
     */
    public RadarOpticTrackerV2 register(String key, String configPath) {
        return trackers.computeIfAbsent(key, k -> {
            System.out.println("[TrackerInvoker] 新建实例: " + key);
            return new RadarOpticTrackerV2(configPath);
        });
    }

    /**
     * 强制重建跟踪器（覆盖已有实例）
     *
     * @param key        实例标识
     * @param configPath 标定参数文件路径
     * @return 新建的跟踪器实例
     */
    public RadarOpticTrackerV2 registerForce(String key, String configPath) {
        System.out.println("[TrackerInvoker] 强制重建: " + key);
        RadarOpticTrackerV2 t = new RadarOpticTrackerV2(configPath);
        trackers.put(key, t);
        return t;
    }

    /**
     * 获取跟踪器
     *
     * @param key 实例标识
     * @return 跟踪器实例，若不存在则返回空 Optional
     */
    public Optional<RadarOpticTrackerV2> get(String key) {
        return Optional.ofNullable(trackers.get(key));
    }

    /**
     * 检查跟踪器是否已注册
     */
    public boolean contains(String key) {
        return trackers.containsKey(key);
    }

    /**
     * 移除跟踪器
     *
     * @param key 实例标识
     * @return 被移除的实例，若不存在则返回 null
     */
    public RadarOpticTrackerV2 remove(String key) {
        return trackers.remove(key);
    }

    /**
     * 计算光电驱动指令
     *
     * @param key         跟踪器标识（必须已注册）
     * @param targetB      目标纬度
     * @param targetL      目标经度
     * @param targetH      目标高度（米）
     * @param radarRange   雷达斜距（米），可为 null
     * @param radarArea    雷达目标面积（平方米），可为 null
     * @return 跟踪命令（返回 RadarOpticTrackerV2.TrackingCommand）
     * @throws IllegalArgumentException 若 key 未注册
     */
    public TrackingCommand compute(String key, double targetB, double targetL, double targetH,
                                                        Double radarRange, Double radarArea) {
        RadarOpticTrackerV2 t = trackers.get(key);
        if (t == null) {
            throw new IllegalArgumentException("Tracker 未注册: " + key);
        }
        return t.compute(targetB, targetL, targetH, radarRange, radarArea);
    }

    /**
     * 列出所有已注册的 key
     */
    public Set<String> listKeys() {
        return Set.copyOf(trackers.keySet());
    }

    /**
     * 清空所有跟踪器实例
     */
    public void clear() {
        trackers.clear();
    }
}
