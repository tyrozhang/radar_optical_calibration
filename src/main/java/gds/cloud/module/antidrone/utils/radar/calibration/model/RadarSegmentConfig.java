package gds.cloud.module.antidrone.utils.radar.calibration.model;

/**
 * 雷达分段配置（带补偿值）
 *
 * 继承基础分段配置，额外包含雷达补偿参数 dAz, dEl
 */
public class RadarSegmentConfig extends SegmentConfig {
    private final double dAz;
    private final double dEl;

    public RadarSegmentConfig(String name, double min, double max, double dAz, double dEl) {
        super(name, min, max);
        this.dAz = dAz;
        this.dEl = dEl;
    }

    public double dAz() { return dAz; }
    public double dEl() { return dEl; }

    @Override
    public String toString() {
        return String.format("%s[ΔAz=%.4f°, ΔEl=%.4f°]", name(), dAz, dEl);
    }
}
