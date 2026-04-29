package gds.cloud.module.antidrone.utils.radar.calibration.model;

/**
 * 分段配置（公共模型）
 *
 * 用于雷达误差分段补偿等场景，支持配置驱动的分段查找。
 */
public class SegmentConfig {
    private final String name;
    private final double min;
    private final double max;

    public SegmentConfig(String name, double min, double max) {
        this.name = name;
        this.min = min;
        this.max = max;
    }

    public String name() { return name; }
    public double min() { return min; }
    public double max() { return max; }

    /**
     * 检查值是否在该段范围内
     * 修复：特殊处理 max = Double.MAX_VALUE 的情况
     */
    public boolean contains(double value) {
        if (max == Double.MAX_VALUE) {
            return value >= min;
        }
        return value >= min && value < max;
    }

    /**
     * 计算段中心值
     */
    public double center() {
        return max == Double.MAX_VALUE ? min : (min + max) / 2;
    }

    @Override
    public String toString() {
        if (max == Double.MAX_VALUE) {
            return String.format("%s[%.1f, +∞)", name, min);
        }
        return String.format("%s[%.1f, %.1f)", name, min, max);
    }
}
