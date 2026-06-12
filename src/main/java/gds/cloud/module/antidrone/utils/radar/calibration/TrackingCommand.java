package gds.cloud.module.antidrone.utils.radar.calibration;

/**
 * 光电跟踪命令（统一数据结构）
 *
 * 字段覆盖 RadarOpticTrackerV2 和 SmoothTracker 的全部输出：
 * - 控制指令：azCmd, elCmd
 * - 中间量：azGeo, elGeo, dAz0, dEl0, dAzRadar, dElRadar
 * - 目标信息：opticalToTargetRange, targetWidth, targetHeight
 * - 时间戳：timestamp（平滑预测场景使用，即时计算场景默认为 0）
 *
 * 约定说明：
 * - azCmd / elCmd / azGeo / elGeo：已按设备 ElevationConvention 输出
 *   （若设备约定为 UP_POSITIVE，俯仰角已做取反翻转）
 * - dAz0 / dEl0 / dAzRadar / dElRadar：始终使用内部约定（up=negative），不做翻转
 *   （补偿值在内部约定下标定，与设备无关）
 */
public record TrackingCommand(
    double azCmd,               // 方位角命令（度）
    double elCmd,               // 俯仰角命令（度），已按设备约定输出
    double azGeo,               // 几何方位角（度）
    double elGeo,               // 几何俯仰角（度），已按设备约定输出
    double dAz0,                // 方位角固定偏差（度），内部约定
    double dEl0,                // 俯仰角固定偏差（度），内部约定（up=negative）
    double dAzRadar,            // 方位角雷达补偿（度），内部约定
    double dElRadar,            // 俯仰角雷达补偿（度），内部约定（up=negative）
    double opticalToTargetRange,// 光电到目标距离（米）
    double targetWidth,         // 目标宽（米）
    double targetHeight,        // 目标高（米）
    long timestamp              // 命令时间戳（毫秒）
) {
    /**
     * 便捷构造：无时间戳（用于即时计算场景，timestamp 默认为 0）
     */
    public TrackingCommand(
        double azCmd, double elCmd,
        double azGeo, double elGeo,
        double dAz0, double dEl0,
        double dAzRadar, double dElRadar,
        double opticalToTargetRange,
        double targetWidth,
        double targetHeight
    ) {
        this(azCmd, elCmd, azGeo, elGeo, dAz0, dEl0, dAzRadar, dElRadar,
             opticalToTargetRange, targetWidth, targetHeight, 0L);
    }
}
