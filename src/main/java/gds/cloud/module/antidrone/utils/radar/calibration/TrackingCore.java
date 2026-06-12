package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.RadarSegmentConfig;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

import java.util.List;

/**
 * 跟踪核心计算逻辑（共享基础类）
 *
 * 从 RadarOpticTrackerV2 和 SmoothTracker 中抽取的公共逻辑：
 * - 雷达补偿类型定义（RadarCompensation / FixedCompensation / SegmentedCompensation）
 * - 雷达补偿值查找
 * - 角度归一化（方位角 0~360°，俯仰角 ±90°）
 * - 基础角度计算（几何角 + 偏差 + 雷达补偿 + 归一化）
 *
 * 类型定义使用普通 interface（非 sealed），允许外部实现，
 * 以便 RadarOpticTrackerV2 中的 record 可以同时实现 V2 的 sealed interface 和此处的 interface，
 * 保持向后兼容。
 */
public class TrackingCore {

    // ==================== 常量定义 ====================

    /** 目标宽默认值（米），radarArea 未传入时使用 */
    public static final double DEFAULT_TARGET_WIDTH = 0.35;

    /** 目标高默认值（米），radarArea 未传入时使用 */
    public static final double DEFAULT_TARGET_HEIGHT = 0.01;

    // ==================== 目标尺寸计算 ====================

    /**
     * 根据雷达面积计算目标宽高
     *
     * @param radarArea 雷达探测的目标面积（平方米），可为 null
     * @return [targetWidth, targetHeight]（米）
     */
    public static double[] computeTargetSize(Double radarArea) {
        if (radarArea != null && radarArea > 0) {
            double sqrtArea = Math.sqrt(radarArea);
            double targetWidth = Math.round(sqrtArea * 100.0) / 100.0;  // 保留2位小数
            double targetHeight = targetWidth;                           // 宽高相同
            return new double[]{targetWidth, targetHeight};
        }
        return new double[]{DEFAULT_TARGET_WIDTH, DEFAULT_TARGET_HEIGHT};
    }

    // ==================== 雷达补偿类型定义 ====================

    /**
     * 雷达补偿策略（基础接口，非 sealed，允许外部实现）
     */
    public interface RadarCompensation {}

    /**
     * 固定雷达补偿（基础接口）
     */
    public interface FixedCompensation extends RadarCompensation {
        double dAz();
        double dEl();
    }

    /**
     * 分段雷达补偿（基础接口）
     */
    public interface SegmentedCompensation extends RadarCompensation {
        List<RadarSegmentConfig> segments();
    }

    // ==================== 角度归一化 ====================

    /**
     * 方位角归一化到 [0°, 360°)
     */
    public static double normalizeAz(double az) {
        az = az % 360.0;
        return (az + 360.0) % 360.0;
    }

    /**
     * 俯仰角归一化到 [-90°, +90°]
     *
     * 约定：向上（仰视）为负，向下（俯视）为正
     * 折叠逻辑：超出范围时反射
     * 例如：120°（俯视120°）→ -60°（仰视60°）
     */
    public static double normalizeEl(double el) {
        // 第一步：归一化到 [-180, +180]
        el = el % 360.0;
        if (el >= 180.0) {
            el -= 360.0;
        }
        // 第二步：折叠到 [-90, +90]
        if (el > 90.0) {
            el = el - 180.0;
        } else if (el < -90.0) {
            el = el + 180.0;
        }
        return el;
    }

    // ==================== 雷达补偿查找 ====================

    /**
     * 根据目标位置获取雷达补偿值
     *
     * @param targetB    目标纬度
     * @param targetL    目标经度
     * @param opticalBlh 光电站BLH（用于计算水平距离）
     * @param radarComp  雷达补偿策略
     * @return [dAz, dEl] 补偿值（度）
     */
    public static double[] getRadarCompensation(double targetB, double targetL,
                                                 StationBLH opticalBlh,
                                                 RadarCompensation radarComp) {
        double dist = CoordinateUtils.haversineDistance(
            targetB, targetL, opticalBlh.B(), opticalBlh.L());

        if (radarComp instanceof FixedCompensation f) {
            return new double[]{f.dAz(), f.dEl()};
        } else if (radarComp instanceof SegmentedCompensation s) {
            for (var seg : s.segments()) {
                if (seg.contains(dist)) {
                    return new double[]{seg.dAz(), seg.dEl()};
                }
            }
            var last = s.segments().get(s.segments().size() - 1);
            return new double[]{last.dAz(), last.dEl()};
        }
        return new double[]{0, 0};
    }

    // ==================== 基础角度计算 ====================

    /**
     * 计算光电跟踪角度（几何角 + 固定偏差 + 雷达补偿 + 归一化）
     *
     * @param targetB    目标纬度
     * @param targetL    目标经度
     * @param targetH    目标高度（米）
     * @param opticalBlh 光电站BLH
     * @param dAz0       方位角固定偏差
     * @param dEl0       俯仰角固定偏差
     * @param radarComp  雷达补偿策略
     * @return [azCmd, elCmd, azGeo, elGeo, dAzRadar, dElRadar] 全部分解量
     */
    public static double[] computeAngles(double targetB, double targetL, double targetH,
                                          StationBLH opticalBlh,
                                          double dAz0, double dEl0,
                                          RadarCompensation radarComp) {
        // 1. 几何角
        double[] azEl = CoordinateUtils.blhToAzEl(
            targetB, targetL, targetH,
            opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
        );

        // 2. 雷达补偿
        double[] radarCompensation = getRadarCompensation(targetB, targetL, opticalBlh, radarComp);
        double dAzRadar = radarCompensation[0];
        double dElRadar = radarCompensation[1];

        // 3. 合成 + 归一化
        // 雷达补偿做减法：从几何角中减去雷达系统误差，得到更真值的角度
        // dAz_radar 定义 = 雷达测量角度 − 真值角度，所以减去它消除误差
        double azCmd = normalizeAz(azEl[0] + dAz0 - dAzRadar);
        double elCmd = normalizeEl(azEl[1] + dEl0 - dElRadar);

        return new double[]{azCmd, elCmd, azEl[0], azEl[1], dAzRadar, dElRadar};
    }
}
