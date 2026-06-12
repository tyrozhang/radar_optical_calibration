package gds.cloud.module.antidrone.utils.radar.calibration;

/**
 * 俯仰角符号约定
 *
 * 不同设备对俯仰角正负方向的定义不同：
 * - UP_NEGATIVE：向上（仰视）为负，向下（俯视）为正（项目内部默认约定）
 * - UP_POSITIVE：向上（仰视）为正，向下（俯视）为负
 *
 * 内部计算始终使用 UP_NEGATIVE 约定，仅在输出 TrackingCommand 时
 * 通过 apply() 方法将俯仰角转换为设备约定的符号方向。
 *
 * 配置示例（calibration_v2.json）：
 *   "elevation_convention": "up_negative"   // 默认，向后兼容
 *   "elevation_convention": "up_positive"   // 向上为正的设备
 */
public enum ElevationConvention {

    /** 向上（仰视）为负，向下（俯视）为正（内部默认约定） */
    UP_NEGATIVE,

    /** 向上（仰视）为正，向下（俯视）为负 */
    UP_POSITIVE;

    /**
     * 按设备约定输出俯仰角
     *
     * 内部始终使用 UP_NEGATIVE（向上为负），若设备使用 UP_POSITIVE，
     * 则对俯仰角取反。
     *
     * @param elInternal 内部计算得到的俯仰角（UP_NEGATIVE 约定）
     * @return 符合设备约定的俯仰角
     */
    public double apply(double elInternal) {
        return this == UP_POSITIVE ? -elInternal : elInternal;
    }

    /**
     * 从配置字符串解析俯仰约定
     *
     * 支持的值（不区分大小写）：
     *   "up_negative" / "up-negative" / "negative" → UP_NEGATIVE（默认）
     *   "up_positive" / "up-positive" / "positive" → UP_POSITIVE
     *
     * @param value 配置字符串，可为 null（默认 UP_NEGATIVE）
     * @return 对应的俯仰约定
     */
    public static ElevationConvention fromString(String value) {
        if (value == null) return UP_NEGATIVE;
        return switch (value.trim().toLowerCase()) {
            case "up_positive", "up-positive", "positive" -> UP_POSITIVE;
            default -> UP_NEGATIVE;
        };
    }
}
