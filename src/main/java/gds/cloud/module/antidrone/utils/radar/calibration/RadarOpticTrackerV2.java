package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.RadarSegmentConfig;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Phase 2 在线跟踪器（配置驱动版）
 *
 * 支持：
 * - 固定雷达补偿
 * - 分段雷达补偿（按距离，配置驱动）
 *
 * 修复：分段查找使用配置边界，避免硬编码
 *
 * 重构说明：
 * - 雷达补偿类型定义同时实现 TrackingCore 的接口，核心计算委托 TrackingCore
 * - 对外 API 完全不变：RadarOpticTrackerV2.RadarCompensation 等类型名、TrackingCommand 字段、compute() 签名均保持原样
 */
public class RadarOpticTrackerV2 {

    private static final Logger log = Logger.getLogger(RadarOpticTrackerV2.class.getName());

    private final StationBLH opticalBlh;
    private final double dAz0;
    private final double dEl0;
    private final RadarCompensation radarComp;
    private final ElevationConvention elevationConvention;
    private final FocalLengthTable focalLengthTable;

    public RadarOpticTrackerV2(String configPath) {
        this(TrackerConfigLoader.load(configPath));
    }

    public RadarOpticTrackerV2(Config config) {
        this.opticalBlh = config.opticalBlh();
        this.dAz0 = config.dAz0();
        this.dEl0 = config.dEl0();
        this.radarComp = config.radarCompensation();
        this.elevationConvention = config.elevationConvention();
        this.focalLengthTable = config.focalLengthTable();
    }

    /** 获取光电站BLH */
    public StationBLH getOpticalBlh() { return opticalBlh; }

    /** 获取方位角固定偏差 */
    public double getDAz0() { return dAz0; }

    /** 获取俯仰角固定偏差 */
    public double getDEl0() { return dEl0; }

    /** 获取雷达补偿策略 */
    public RadarCompensation getRadarCompensation() { return radarComp; }

    /**
     * 计算光电驱动指令（无雷达距离和面积输入）
     */
    public TrackingCommand compute(double targetB, double targetL, double targetH) {
        return compute(targetB, targetL, targetH, null, null);
    }

    /**
     * 计算光电驱动指令（含雷达距离和面积输入）
     *
     * @param radarToTargetRange 雷达探测的目标斜距（米），可为 null
     * @param radarArea 雷达探测的目标面积（平方米），可为 null
     */
    public TrackingCommand compute(double targetB, double targetL, double targetH,
                                   Double radarToTargetRange, Double radarArea) {
        // 1. 使用 TrackingCore 计算角度（几何角 + 偏差 + 雷达补偿 + 归一化）
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

        // 2. 计算光电到目标距离（米，3D斜距）
        StationBLH targetBlh = new StationBLH(targetB, targetL, targetH);
        double opticalToTargetRange = CoordinateUtils.distance3D(opticalBlh, targetBlh);

        // 3. 焦距查表（基于3D斜距，与雷达补偿的 haversine 水平距离不同）
        double focalLength = focalLengthTable.lookup(opticalToTargetRange);

        // 4. 计算目标宽高
        double[] targetSize = TrackingCore.computeTargetSize(radarArea);
        double targetWidth = targetSize[0];
        double targetHeight = targetSize[1];

        TrackingCommand cmd = new TrackingCommand(azCmd, elCmdOut, azGeo, elGeoOut, dAz0, dEl0, dAzRadar, dElRadar, opticalToTargetRange, targetWidth, targetHeight, focalLength);

        log.fine(String.format("=== 计算过程 ==="));
        log.fine(String.format("方位角 (Az): 几何角=%.6f° | 雷达补偿=%.6f° | 固定补偿=%.6f° | 总指令=%.6f°",
            azGeo, dAzRadar, dAz0, azCmd));
        log.fine(String.format("俯仰角 (El): 几何角=%.6f° | 雷达补偿=%.6f° | 固定补偿=%.6f° | 总指令=%.6f° | 设备约定(%s): elCmdOut=%.6f°",
            elGeo, dElRadar, dEl0, elCmd, elevationConvention, elCmdOut));
        log.fine(String.format("距离=%.2fm, 宽=%.2fm, 高=%.2fm", opticalToTargetRange, targetWidth, targetHeight));
        log.fine(String.format("目标BLH: B=%.6f°, L=%.6f°, H=%.1fm", targetB, targetL, targetH));
        log.fine(String.format("光电BLH: B=%.6f°, L=%.6f°, H=%.1fm", opticalBlh.B(), opticalBlh.L(), opticalBlh.H()));
        log.fine(String.format("雷达BLH: B=%.6f°, L=%.6f°, H=%.1fm", opticalBlh.B(), opticalBlh.L(), opticalBlh.H()));

        return cmd;
    }

    // ==================== 向后兼容的类型定义 ====================
    // 保持 RadarOpticTrackerV2.RadarCompensation 等类型名可用
    // 内部 record 同时实现 TrackingCore 的接口，以便 TrackingCore 的静态方法可以识别

    /**
     * 雷达补偿策略
     * 同时继承 TrackingCore.RadarCompensation，使 TrackingCore 的静态方法可识别
     */
    public sealed interface RadarCompensation extends TrackingCore.RadarCompensation
        permits FixedCompensation, SegmentedCompensation {}

    /**
     * 固定雷达补偿
     * 同时实现 TrackingCore.FixedCompensation，使 TrackingCore.getRadarCompensation() 可识别
     */
    public record FixedCompensation(double dAz, double dEl)
        implements RadarCompensation, TrackingCore.FixedCompensation {}

    /**
     * 分段雷达补偿
     * 同时实现 TrackingCore.SegmentedCompensation，使 TrackingCore.getRadarCompensation() 可识别
     */
    public record SegmentedCompensation(List<RadarSegmentConfig> segments)
        implements RadarCompensation, TrackingCore.SegmentedCompensation {}

    public record Config(StationBLH opticalBlh, double dAz0, double dEl0, RadarCompensation radarCompensation, long totalDelayMs, ElevationConvention elevationConvention, FocalLengthTable focalLengthTable) {
        public Config(StationBLH opticalBlh, double dAz0, double dEl0, RadarCompensation radarCompensation, long totalDelayMs, ElevationConvention elevationConvention) {
            this(opticalBlh, dAz0, dEl0, radarCompensation, totalDelayMs, elevationConvention, FocalLengthTable.empty());
        }
        public Config(StationBLH opticalBlh, double dAz0, double dEl0, RadarCompensation radarCompensation, long totalDelayMs) {
            this(opticalBlh, dAz0, dEl0, radarCompensation, totalDelayMs, ElevationConvention.UP_NEGATIVE);
        }
        public Config(StationBLH opticalBlh, double dAz0, double dEl0, RadarCompensation radarCompensation) {
            this(opticalBlh, dAz0, dEl0, radarCompensation, 0L);
        }
    }

    // ==================== 主程序 ====================

    public static void main(String[] args) {
        String configPath = "calibration_v2.json";

        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-c") && i + 1 < args.length) configPath = args[++i];
            if (args[i].equals("--simulate") || args[i].equals("-s")) {
                simulate(configPath);
                return;
            }
        }

        // 交互模式
        RadarOpticTrackerV2 tracker = new RadarOpticTrackerV2(configPath);
        log.info("Phase 2 跟踪器已就绪，等待目标数据...");
        log.info("    输入格式: B L H [radarRange] [radarArea]");
        log.info("    示例: 39.905 116.408 100 5000 2.5");
        log.info("    输入 'q' 退出\n");

        Scanner scanner = new Scanner(System.in);
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine().trim();
            if (line.equalsIgnoreCase("q")) break;

            String[] parts = line.split("\\s+");
            if (parts.length < 3) {
                log.warning("格式错误，请输入: B L H [radarRange] [radarArea]");
                continue;
            }

            try {
                double B = Double.parseDouble(parts[0]);
                double L = Double.parseDouble(parts[1]);
                double H = Double.parseDouble(parts[2]);

                Double radarRange = parts.length > 3 ? Double.parseDouble(parts[3]) : null;
                Double radarArea = parts.length > 4 ? Double.parseDouble(parts[4]) : null;

                TrackingCommand cmd = tracker.compute(B, L, H, radarRange, radarArea);
                log.info(String.format("最终结果: Az=%d  El=%d, 光电距目标=%.2fm, 目标宽=%.2fm, 目标高=%.2fm",
                    (long)cmd.azCmd(), (long)cmd.elCmd(),
                    cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight()));
            } catch (Exception e) {
                log.log(Level.SEVERE, "计算错误: " + e.getMessage(), e);
            }
        }
    }

    private static void simulate(String configPath) {
        log.info("Phase 2 模拟跟踪测试（带雷达参数）");

        RadarOpticTrackerV2 tracker = new RadarOpticTrackerV2(configPath);

        double bStart = 39.9050, lStart = 116.4080, hStart = 100.0;
        double bEnd = 39.9080, lEnd = 116.4110, hEnd = 250.0;

        for (int i = 0; i < 10; i++) {
            double t = i / 9.0;
            double B = bStart + t * (bEnd - bStart);
            double L = lStart + t * (lEnd - lStart);
            double H = hStart + t * (hEnd - hStart);

            Double radarRange = 5000.0 + i * 500;
            Double radarArea = 2.5;

            TrackingCommand cmd = tracker.compute(B, L, H, radarRange, radarArea);

            log.info(String.format("帧%d: B=%.6f H=%.1f Az=%.4f El=%.4f dist=%.2f 宽=%.2f 高=%.2f",
                i + 1, B, H, cmd.azCmd(), cmd.elCmd(),
                cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight()));
        }

        log.info("模拟完成（无雷达面积参数测试）");

        TrackingCommand cmdDefault = tracker.compute(39.9050, 116.4080, 100.0, null, null);
        log.info(String.format("无雷达面积参数测试: 宽=%.2fm, 高=%.2fm",
            cmdDefault.targetWidth(), cmdDefault.targetHeight()));
    }

    public static class TrackingException extends RuntimeException {
        public TrackingException(String msg) { super(msg); }
        public TrackingException(String msg, Throwable cause) { super(msg, cause); }
    }
}
