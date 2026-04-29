package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.util.SimpleJsonParser;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * 标定参数反向验证测试
 *
 * 功能：
 * 1. 读取 calibration_data_v2.json 中的雷达BLH数据点
 * 2. 使用 RadarOpticTrackerV2 计算理论 Az/El
 * 3. 与实测 Az_measured/El_measured 对比
 * 4. 输出误差统计，验证标定参数质量
 */
public class CalibrationReverseTest {

    private final RadarOpticTrackerV2 tracker;
    private final List<DataPoint> dataPoints;
    private final Map<String, Object> calibrationData;
    private final String signalType;

    public CalibrationReverseTest(String calibrationDataPath, String calibrationConfigPath, String signalType) throws IOException {
        // 加载在线跟踪器（使用标定后的参数）
        this.tracker = new RadarOpticTrackerV2(calibrationConfigPath);

        // 读取原始标定数据
        String json = Files.readString(Path.of(calibrationDataPath));
        this.calibrationData = SimpleJsonParser.parse(json);

        // 设置信号类型
        this.signalType = signalType;

        // 解析数据点
        this.dataPoints = parseDataPoints();
    }

    @SuppressWarnings("unchecked")
    private List<DataPoint> parseDataPoints() {
        if (!"radar_blh".equals(signalType) && !"target_blh".equals(signalType)) {
            throw new IllegalArgumentException("signalType 仅支持 radar_blh 或 target_blh，当前值: " + signalType);
        }
        List<DataPoint> points = new ArrayList<>();
        List<Map<String, Object>> rawPoints = (List<Map<String, Object>>) calibrationData.get("data_points");
        System.out.println(">》》》》》》》》》使用标定数据" + signalType + "---------");
        for (Map<String, Object> raw : rawPoints) {
            Map<String, Object> radarBlh = (Map<String, Object>) raw.get(signalType);
            DataPoint point = new DataPoint(
                toDouble(radarBlh.get("B")),
                toDouble(radarBlh.get("L")),
                toDouble(radarBlh.get("H")),
                toDouble(raw.get("Az_measured")),
                toDouble(raw.get("El_measured")),
                (String) raw.get("target_id"),
                (String) raw.get("timestamp")
            );
            points.add(point);
        }

        return points;
    }

    /**
     * 执行验证测试
     */
    public ValidationResult validate() {
        List<PointResult> results = new ArrayList<>();

        for (int i = 0; i < dataPoints.size(); i++) {
            DataPoint point = dataPoints.get(i);

            // 使用在线跟踪器计算理论值
            TrackingCommand cmd = tracker.compute(
                point.B, point.L, point.H, null, null
            );

            // 计算误差
            double azError = normalizeAngleDiff(cmd.azCmd(), point.azMeasured);
            double elError = cmd.elCmd() - point.elMeasured;

            PointResult result = new PointResult(
                point, cmd.azCmd(), cmd.elCmd(), azError, elError,
                cmd.azGeo(), cmd.elGeo()
            );
            results.add(result);
        }

        // 输出结果
        System.out.println("\n" + "=".repeat(100));
        System.out.println("标定参数反向验证测试");
        System.out.println("=".repeat(100));
        System.out.printf("%-4s %-12s %-12s %-12s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-10s%n",
            "序号", "B", "L", "H", "Az_实测", "Az_计算", "Az_误差", "El_实测", "El_计算", "El_误差", "Az_几何", "El_几何");
        System.out.println("-".repeat(100));

        for (int i = 0; i < results.size(); i++) {
            PointResult result = results.get(i);
            DataPoint point = result.point;
            System.out.printf("%-4d %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f %-12.4f%n",
                i + 1,
                point.B, point.L, point.H,
                point.azMeasured, result.azComputed, result.azError,
                point.elMeasured, result.elComputed, result.elError,
                result.azGeo, result.elGeo
            );
        }

        // 计算统计指标
        ValidationResult validation = calculateStatistics(results);

        // 输出统计结果
        printStatistics(validation);

        return validation;
    }

    /**
     * 计算角度差（处理 0/360 跨越）
     */
    private double normalizeAngleDiff(double computed, double measured) {
        double diff = computed - measured;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    private ValidationResult calculateStatistics(List<PointResult> results) {
        double[] azErrors = results.stream().mapToDouble(r -> r.azError).toArray();
        double[] elErrors = results.stream().mapToDouble(r -> r.elError).toArray();

        return new ValidationResult(
            results,
            calculateStats(azErrors),
            calculateStats(elErrors)
        );
    }

    private ErrorStats calculateStats(double[] errors) {
        int n = errors.length;
        double sum = 0, sumSq = 0;
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;

        for (double e : errors) {
            sum += e;
            sumSq += e * e;
            max = Math.max(max, Math.abs(e));
            min = Math.min(min, Math.abs(e));
        }

        double mean = sum / n;
        double variance = sumSq / n - mean * mean;
        double std = Math.sqrt(Math.max(0, variance));
        double rms = Math.sqrt(sumSq / n);

        return new ErrorStats(mean, std, rms, max, min, n);
    }

    private void printStatistics(ValidationResult result) {
        System.out.println("\n" + "=".repeat(100));
        System.out.println("误差统计结果");
        System.out.println("=".repeat(100));

        System.out.println("\n【方位角 (Az) 误差】");
        printErrorStats(result.azStats);

        System.out.println("\n【俯仰角 (El) 误差】");
        printErrorStats(result.elStats);

        // 评估结论
        System.out.println("\n" + "=".repeat(100));
        System.out.println("验证结论");
        System.out.println("=".repeat(100));

        double azRms = result.azStats.rms;
        double elRms = result.elStats.rms;

        System.out.printf("方位角 RMS 误差: %.4f°%n", azRms);
        System.out.printf("俯仰角 RMS 误差: %.4f°%n", elRms);

        if (azRms < 0.5 && elRms < 0.5) {
            System.out.println("✓ 标定质量：优秀 (RMS < 0.5°)");
        } else if (azRms < 1.0 && elRms < 1.0) {
            System.out.println("✓ 标定质量：良好 (RMS < 1.0°)");
        } else if (azRms < 2.0 && elRms < 2.0) {
            System.out.println("△ 标定质量：一般 (RMS < 2.0°)");
        } else {
            System.out.println("✗ 标定质量：较差 (RMS ≥ 2.0°)，建议重新标定");
        }

        // 野值检测
        List<PointResult> outliers = detectOutliers(result.pointResults);
        if (!outliers.isEmpty()) {
            System.out.println("\n⚠ 检测到潜在野值点：");
            for (PointResult o : outliers) {
                System.out.printf("   - %s: Az误差=%.4f°, El误差=%.4f°%n",
                    o.point.targetId, o.azError, o.elError);
            }
        }
    }

    private void printErrorStats(ErrorStats stats) {
        System.out.printf("  均值:     %+.4f°%n", stats.mean);
        System.out.printf("  标准差:   %.4f°%n", stats.std);
        System.out.printf("  RMS:      %.4f°%n", stats.rms);
        System.out.printf("  最大绝对值: %.4f°%n", stats.maxAbs);
        System.out.printf("  最小绝对值: %.4f°%n", stats.minAbs);
    }

    /**
     * 使用 3σ 准则检测野值
     */
    private List<PointResult> detectOutliers(List<PointResult> results) {
        double[] azErrors = results.stream().mapToDouble(r -> Math.abs(r.azError)).toArray();
        double[] elErrors = results.stream().mapToDouble(r -> Math.abs(r.elError)).toArray();

        double azMean = Arrays.stream(azErrors).average().orElse(0);
        double elMean = Arrays.stream(elErrors).average().orElse(0);
        double azStd = calculateStd(azErrors, azMean);
        double elStd = calculateStd(elErrors, elMean);

        double azThreshold = azMean + 3 * azStd;
        double elThreshold = elMean + 3 * elStd;

        List<PointResult> outliers = new ArrayList<>();
        for (PointResult r : results) {
            if (Math.abs(r.azError) > azThreshold || Math.abs(r.elError) > elThreshold) {
                outliers.add(r);
            }
        }

        return outliers;
    }

    private double calculateStd(double[] values, double mean) {
        double sumSq = 0;
        for (double v : values) {
            sumSq += (v - mean) * (v - mean);
        }
        return Math.sqrt(sumSq / values.length);
    }

    private double toDouble(Object obj) {
        if (obj instanceof Number n) return n.doubleValue();
        return Double.parseDouble(obj.toString());
    }

    private String truncate(String s, int maxLen) {
        if (s == null) return "";
        return s.length() > maxLen ? s.substring(0, maxLen - 3) + "..." : s;
    }

    // ==================== 数据结构 ====================

    private record DataPoint(
        double B, double L, double H,
        double azMeasured, double elMeasured,
        String targetId, String timestamp
    ) {}

    private record PointResult(
        DataPoint point,
        double azComputed, double elComputed,
        double azError, double elError,
        double azGeo, double elGeo
    ) {}

    public record ErrorStats(
        double mean, double std, double rms,
        double maxAbs, double minAbs, int count
    ) {}

    public record ValidationResult(
        List<PointResult> pointResults,
        ErrorStats azStats, ErrorStats elStats
    ) {}

    // ==================== 主程序 ====================

    public static void main(String[] args) {
        String dataPath = "calibration_data_v2.json";
        String configPath = "calibration_v2.json";
        String signalType = "target_blh";

        // 解析命令行参数
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-d") && i + 1 < args.length) {
                dataPath = args[++i];
            } else if (args[i].equals("-c") && i + 1 < args.length) {
                configPath = args[++i];
            } else if (args[i].equals("-s") && i + 1 < args.length) {
                signalType = args[++i];
            }
        }

        System.out.println("[*] 标定参数反向验证器");
        System.out.printf("    数据文件: %s%n", dataPath);
        System.out.printf("    配置文件: %s%n", configPath);
        System.out.printf("    信号类型: %s%n", signalType);

        try {
            CalibrationReverseTest validator = new CalibrationReverseTest(dataPath, configPath, signalType);
            validator.validate();
        } catch (IOException e) {
            System.err.println("错误: 文件读取失败 - " + e.getMessage());
            System.exit(1);
        } catch (Exception e) {
            System.err.println("错误: 验证过程异常 - " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }
}
