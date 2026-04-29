package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.RadarSegmentConfig;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

import java.util.*;

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

    private final StationBLH opticalBlh;
    private final double dAz0;
    private final double dEl0;
    private final RadarCompensation radarComp;

    public RadarOpticTrackerV2(String configPath) {
        this(TrackerConfigLoader.load(configPath));
    }

    public RadarOpticTrackerV2(Config config) {
        this.opticalBlh = config.opticalBlh();
        this.dAz0 = config.dAz0();
        this.dEl0 = config.dEl0();
        this.radarComp = config.radarCompensation();
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

        // 2. 计算光电到目标距离（米）
        StationBLH targetBlh = new StationBLH(targetB, targetL, targetH);
        double opticalToTargetRange = CoordinateUtils.distance3D(opticalBlh, targetBlh);

        // 3. 计算目标宽高
        double[] targetSize = TrackingCore.computeTargetSize(radarArea);
        double targetWidth = targetSize[0];
        double targetHeight = targetSize[1];

        TrackingCommand cmd = new TrackingCommand(azCmd, elCmd, azGeo, elGeo, dAz0, dEl0, dAzRadar, dElRadar, opticalToTargetRange, targetWidth, targetHeight);

        // 调试打印
        System.out.println("=== 计算过程 ===");

        System.out.println("方位角 (Az):");
        System.out.printf("  几何角: azGeo=%.6f° | 雷达补偿: dAzRadar=%.6f° | 固定补偿: dAz0=%.6f° | 总指令 (归一化前): azCmd=%.6f°%n",
            azGeo, dAzRadar, dAz0, azCmd);
        System.out.println("俯仰角 (El):");
        System.out.printf("  几何角: elGeo=%.6f° | 雷达补偿: dElRadar=%.6f° | 固定补偿: dEl0=%.6f° | 总指令 (归一化前): elCmd=%.6f°%n",
            elGeo, dElRadar, dEl0, elCmd);
        System.out.printf("其他: 距离=%.2fm, 宽=%.2fm, 高=%.2fm%n", opticalToTargetRange, targetWidth, targetHeight);
        System.out.println("=== 相关目标BLH信息 ===");
        System.out.printf("目标BLH: B=%.6f°, L=%.6f°, H=%.1fm%n", targetB, targetL, targetH);
        System.out.printf("光电BLH: B=%.6f°, L=%.6f°, H=%.1fm%n", opticalBlh.B(), opticalBlh.L(), opticalBlh.H());
        System.out.printf("雷达BLH: B=%.6f°, L=%.6f°, H=%.1fm%n", opticalBlh.B(), opticalBlh.L(), opticalBlh.H());  


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

    public record Config(StationBLH opticalBlh, double dAz0, double dEl0, RadarCompensation radarCompensation, long totalDelayMs) {
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
        System.out.println("\n[*] Phase 2 跟踪器已就绪，等待目标数据...");
        System.out.println("    输入格式: B L H [radarRange] [radarArea]");
        System.out.println("    示例: 39.905 116.408 100 5000 2.5");
        System.out.println("    输入 'q' 退出\n");

        Scanner scanner = new Scanner(System.in);
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine().trim();
            if (line.equalsIgnoreCase("q")) break;

            String[] parts = line.split("\\s+");
            if (parts.length < 3) {
                System.out.println("    格式错误，请输入: B L H [radarRange] [radarArea]");
                continue;
            }

            try {
                double B = Double.parseDouble(parts[0]);
                double L = Double.parseDouble(parts[1]);
                double H = Double.parseDouble(parts[2]);
                
                Double radarRange = parts.length > 3 ? Double.parseDouble(parts[3]) : null;
                Double radarArea = parts.length > 4 ? Double.parseDouble(parts[4]) : null;

                TrackingCommand cmd = tracker.compute(B, L, H, radarRange, radarArea);
                System.out.println(" ------最终结果");
                System.out.printf("    → Az=%d  El=%d%n",
                    (long)cmd.azCmd(), (long)cmd.elCmd());
                System.out.printf("      光电距目标: %.2fm | 目标宽: %.2fm | 目标高: %.2fm%n",
                    cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight());
            } catch (Exception e) {
                System.out.println("    错误: " + e.getMessage());
            }
        }
    }

    private static void simulate(String configPath) {
        System.out.println("\n[*] Phase 2 模拟跟踪测试（带雷达参数）\n");

        RadarOpticTrackerV2 tracker = new RadarOpticTrackerV2(configPath);

        double bStart = 39.9050, lStart = 116.4080, hStart = 100.0;
        double bEnd = 39.9080, lEnd = 116.4110, hEnd = 250.0;

        System.out.printf("  %4s  %10s  %8s  %6s  %6s  %10s  %6s  %6s%n",
            "帧号", "B", "H", "Az", "El", "optDist(m)", "宽(m)", "高(m)");
        System.out.println("  " + "-".repeat(80));

        for (int i = 0; i < 10; i++) {
            double t = i / 9.0;
            double B = bStart + t * (bEnd - bStart);
            double L = lStart + t * (lEnd - lStart);
            double H = hStart + t * (hEnd - hStart);

            // 模拟雷达参数
            Double radarRange = 5000.0 + i * 500;  // 5km-9.5km 递增
            Double radarArea = 2.5;                  // 固定面积

            TrackingCommand cmd = tracker.compute(B, L, H, radarRange, radarArea);

            System.out.printf("  %4d  %10.6f  %8.1f  %10.4f  %10.4f  %10.2f  %6.2f  %6.2f%n",
                i + 1, B, H, cmd.azCmd(), cmd.elCmd(),
                cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight());
        }

        System.out.println("\n[*] 模拟完成（无雷达面积参数测试）");
        
        // 无雷达面积参数测试
        System.out.println("\n[*] 无雷达面积参数测试（使用默认值）\n");
        TrackingCommand cmdDefault = tracker.compute(39.9050, 116.4080, 100.0, null, null);
        System.out.printf("  输入: B=39.9050, L=116.4080, H=100.0, radarRange=null, radarArea=null%n");
        System.out.printf("  输出: 宽=%.2fm, 高=%.2fm%n", 
            cmdDefault.targetWidth(), cmdDefault.targetHeight());
    }

    public static class TrackingException extends RuntimeException {
        public TrackingException(String msg) { super(msg); }
        public TrackingException(String msg, Throwable cause) { super(msg, cause); }
    }
}
