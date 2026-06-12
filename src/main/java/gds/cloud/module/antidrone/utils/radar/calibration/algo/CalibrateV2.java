package gds.cloud.module.antidrone.utils.radar.calibration.algo;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;
import gds.cloud.module.antidrone.utils.radar.calibration.util.SimpleJsonParser;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.util.*;

/**
 * Phase 2 标定程序（修复版）
 *
 * 输出 calibration_v2.json，包含：
 * - 原有光电标定参数（ΔAz0, ΔEl0）
 * - 雷达补偿参数（固定或分段）
 * - 雷达误差分析报告
 */
public class CalibrateV2 {

    private static final String JSON_FOLDER = "json";

    public static void main(String[] args) throws IOException {
        String inputPath = "calibration_data_v2.json";
        String outputPath = "calibration_v2.json";

        CalibrateV2.process(inputPath, outputPath);
    }

    /**
     * 生成标定文件（json目录下）
     *
     * @param inputPath  输入文件名（如 "calibration_data_v2.json"）
     * @param outputPath 输出文件名（如 "calibration_v2.json" ）
     * @throws IOException 如果文件读写失败
     */
    public static void generateCalibrateFile(String inputPath, String outputPath) throws IOException {
        String resolvedInput = resolvePath(inputPath);
        String resolvedOutput = resolvePath(outputPath);
        process(resolvedInput, resolvedOutput);
    }

    /**
     * 核心标定处理流程
     *
     * @param inputPath  完整输入路径
     * @param outputPath 完整输出路径
     * @throws IOException 如果文件读写失败
     */
    public static void process(String inputPath, String outputPath) throws IOException {
        System.out.println("[*] Phase 2 标定程序");
        System.out.println("    输入: " + inputPath);
        System.out.println("    输出: " + outputPath);

        // 1. 加载 Phase 2 格式数据
        System.out.println("[*] 加载 Phase 2 标定数据...");
        CalibrateV2.Phase2Data data = loadPhase2Data(inputPath);
        System.out.println("    共 " + data.points().size() + " 个数据点");

        // 2. 光电静态偏差标定
        System.out.println("[*] 计算光电静态偏差...");
        CalibrateV2.CalibrationResult optical = calibrateOptical(data);
        System.out.printf("    ΔAz0 = %+.6f°, ΔEl0 = %+.6f°%n", optical.dAz0(), optical.dEl0());

        // 3. 雷达误差分析
        System.out.println("[*] 分析雷达误差...");
        StationBLH radarStation = data.radarBlh();
        StationBLH opticalBlh = data.opticalBlh();
        List<RadarErrorAnalyzer.ErrorSample> errorSamples = data.points().stream()
                .filter(p -> p.radarBlh() != null)
                .map(p -> {
                    double distKm = CoordinateUtils.distance3D(radarStation, p.targetBlh()) / 1000.0;

                    double[] opticalToRadar = CoordinateUtils.blhToAzEl(
                            p.radarBlh().B(), p.radarBlh().L(), p.radarBlh().H(),
                            opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
                    );
                    double[] opticalToTrue = CoordinateUtils.blhToAzEl(
                            p.targetBlh().B(), p.targetBlh().L(), p.targetBlh().H(),
                            opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
                    );
                    double azError = normalizeAngle(opticalToRadar[0] - opticalToTrue[0]);
                    double elError = opticalToRadar[1] - opticalToTrue[1];
                    return new RadarErrorAnalyzer.ErrorSample(distKm, p.targetBlh().H(), azError, elError);
                })
                .toList();

        RadarErrorAnalyzer analyzer = new RadarErrorAnalyzer();
        RadarErrorAnalyzer.RadarAnalysisResult radarAnalysis = analyzer.analyze(errorSamples);

        // 4. 构建 Phase 2 结果
        CalibrateV2.CalibrationResultV2 result = new CalibrateV2.CalibrationResultV2(
                optical.dAz0(), optical.dEl0(),
                optical.rmsAz(), optical.rmsEl(),
                optical.rawRmsAz(), optical.rawRmsEl(),
                optical.validCount(), optical.totalCount(),
                radarAnalysis.strategy(),
                radarAnalysis
        );

        // 5. 保存结果
        saveCalibrationV2(result, data, outputPath);

        // 6. 打印报告
        printReport(optical, radarAnalysis);

        System.out.println("[*] Phase 2 标定完成: " + outputPath);
    }

    /**
     * 解析路径为 json 文件夹下的完整路径
     */
    private static String resolvePath(String relativePath) {
        String baseDir = System.getProperty("user.dir");
        return baseDir + "/" + JSON_FOLDER + "/" + relativePath;
    }

    /**
     * 光电静态偏差标定
     *
     * 流程：计算每点偏差 → 3σ 野值剔除 → 加权平均 → RMS
     */
    private static CalibrateV2.CalibrationResult calibrateOptical(CalibrateV2.Phase2Data data) {
        List<Double> dAz0List = new ArrayList<>();
        List<Double> dEl0List = new ArrayList<>();
        List<Double> weights = new ArrayList<>();

        StationBLH opticalBlh = data.opticalBlh();

        for (int i = 0; i < data.points().size(); i++) {
            CalibrateV2.SimulatedPoint p = data.points().get(i);
            double[] geoAzEl = CoordinateUtils.blhToAzEl(
                    p.targetBlh().B(), p.targetBlh().L(), p.targetBlh().H(),
                    opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
            );

            // DEBUG: 打印每个数据点的详细信息
            System.out.printf("    [Point %d] target=(%.6f, %.6f, %.6f) geoAz=%.6f geoEl=%.6f | Az_measured=%.2f El_measured=%.2f | dAz0=%.6f dEl0=%.6f%n",
                    i, p.targetBlh().B(), p.targetBlh().L(), p.targetBlh().H(),
                    geoAzEl[0], geoAzEl[1], p.azMeasured(), p.elMeasured(),
                    normalizeAngle(p.azMeasured() - geoAzEl[0]), p.elMeasured() - geoAzEl[1]);

            // 计算零点偏差
            double dAz0 = normalizeAngle(p.azMeasured() - geoAzEl[0]);
            double dEl0 = p.elMeasured() - geoAzEl[1];

            dAz0List.add(dAz0);
            dEl0List.add(dEl0);

            // 加权：近距离权重更大
            double dist3D = CoordinateUtils.distance3D(opticalBlh, p.targetBlh()) / 1000.0;
            weights.add(1.0 / (1.0 + dist3D));
        }

        // 先算原始 RMS（剔除前，用于诊断）
        double rawMeanAz = weightedAverage(dAz0List, weights);
        double rawMeanEl = weightedAverage(dEl0List, weights);
        double rawRmsAz = calculateRMS(dAz0List, rawMeanAz);
        double rawRmsEl = calculateRMS(dEl0List, rawMeanEl);

        // 3σ 野值剔除（方位/俯仰独立）
        CalibrateV2.OutlierFilterResult azFilter = filterOutliers3Sigma(dAz0List, weights);
        CalibrateV2.OutlierFilterResult elFilter = filterOutliers3Sigma(dEl0List, weights);

        System.out.println("    方位野值剔除: " + dAz0List.size() + " → " + azFilter.validValues.size()
                + " (剔除 " + azFilter.removedCount + " 个)");
        System.out.println("    俯仰野值剔除: " + dEl0List.size() + " → " + elFilter.validValues.size()
                + " (剔除 " + elFilter.removedCount + " 个)");

        // 用剔除后的数据做加权平均
        double dAz0 = weightedAverage(azFilter.validValues, azFilter.validWeights);
        double dEl0 = weightedAverage(elFilter.validValues, elFilter.validWeights);

        // RMS 用剔除后数据计算
        int validCount = Math.min(azFilter.validValues.size(), elFilter.validValues.size());
        double rmsAz = calculateRMS(azFilter.validValues, dAz0);
        double rmsEl = calculateRMS(elFilter.validValues, dEl0);

        return new CalibrateV2.CalibrationResult(dAz0, dEl0, rmsAz, rmsEl,
                rawRmsAz, rawRmsEl, validCount, dAz0List.size());
    }

    /**
     * 角度归一化到 [-180, 180) 区间
     */
    private static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle >= 180) angle -= 360;
        else if (angle < -180) angle += 360;
        return angle;
    }

    private static double weightedAverage(List<Double> values, List<Double> weights) {
        double sum = 0, weightSum = 0;
        for (int i = 0; i < values.size(); i++) {
            sum += values.get(i) * weights.get(i);
            weightSum += weights.get(i);
        }
        if (weightSum < 1e-10) {
            return values.stream().mapToDouble(v -> v).average().orElse(0.0);
        }
        return sum / weightSum;
    }

    private static double calculateRMS(List<Double> values, double mean) {
        double sumSq = values.stream()
                .mapToDouble(v -> Math.pow(v - mean, 2))
                .sum();
        return Math.sqrt(sumSq / values.size());
    }

    // ==================== 野值剔除 ====================

    private record OutlierFilterResult(
            List<Double> validValues, List<Double> validWeights, int removedCount) {}

    /**
     * 3σ 野值剔除（迭代式）
     *
     * 剔除条件：加权标准差下 |value - weightedMean| > 3 × weightedStd
     * 迭代：剔除后重新计算 mean/std，最多 3 轮
     * 保护：
     *   - n < 10 时直接返回全保留
     *   - 每轮至少保留 60% 的样本
     */
    private static CalibrateV2.OutlierFilterResult filterOutliers3Sigma(
            List<Double> values, List<Double> weights) {

        if (values.size() < 10) {
            return new CalibrateV2.OutlierFilterResult(new ArrayList<>(values), new ArrayList<>(weights), 0);
        }

        List<Integer> validIndices = new ArrayList<>();
        for (int i = 0; i < values.size(); i++) validIndices.add(i);

        int removedTotal = 0;

        for (int round = 0; round < 3; round++) {
            double mean = 0, sumW = 0;
            for (int idx : validIndices) {
                mean += values.get(idx) * weights.get(idx);
                sumW += weights.get(idx);
            }
            if (sumW < 1e-10) break;
            mean /= sumW;

            double var = 0, sumW2 = 0;
            for (int idx : validIndices) {
                double d = values.get(idx) - mean;
                var += weights.get(idx) * d * d;
                sumW2 += weights.get(idx);
            }
            double std = Math.sqrt(var / sumW2);

            List<Integer> newValid = new ArrayList<>();
            int removed = 0;
            for (int idx : validIndices) {
                if (Math.abs(values.get(idx) - mean) > 3 * std) {
                    removed++;
                } else {
                    newValid.add(idx);
                }
            }

            if (removed == 0) break;
            removedTotal += removed;

            if (newValid.size() < values.size() * 0.6) {
                break;
            }

            validIndices = newValid;
        }

        List<Double> validValues = new ArrayList<>();
        List<Double> validWeights = new ArrayList<>();
        for (int idx : validIndices) {
            validValues.add(values.get(idx));
            validWeights.add(weights.get(idx));
        }

        return new CalibrateV2.OutlierFilterResult(validValues, validWeights, removedTotal);
    }

    private static CalibrateV2.Phase2Data loadPhase2Data(String path) throws IOException {
        String json = Files.readString(Path.of(path));
        Map<String, Object> raw = SimpleJsonParser.parse(json);

        @SuppressWarnings("unchecked")
        Map<String, Object> radarMap = (Map<String, Object>) raw.get("radar_blh");
        @SuppressWarnings("unchecked")
        Map<String, Object> opticalMap = (Map<String, Object>) raw.get("optical_blh");

        StationBLH radarBlh = parseBLH(radarMap);
        StationBLH opticalBlh = parseBLH(opticalMap);

        @SuppressWarnings("unchecked")
        List<Object> pointsRaw = (List<Object>) raw.get("data_points");
        List<CalibrateV2.SimulatedPoint> points = new ArrayList<>();

        for (Object obj : pointsRaw) {
            @SuppressWarnings("unchecked")
            Map<String, Object> pt = (Map<String, Object>) obj;
            @SuppressWarnings("unchecked")
            Map<String, Object> tb = (Map<String, Object>) pt.get("target_blh");
            @SuppressWarnings("unchecked")
            Map<String, Object> rb = pt.containsKey("radar_blh")
                    ? (Map<String, Object>) pt.get("radar_blh") : null;

            points.add(new CalibrateV2.SimulatedPoint(
                    parseBLH(tb),
                    rb != null ? parseBLH(rb) : null,
                    toDouble(pt.get("Az_measured")),
                    toDouble(pt.get("El_measured")),
                    pt.containsKey("timestamp") ? pt.get("timestamp").toString() : null,
                    pt.containsKey("target_id") ? pt.get("target_id").toString() : null,
                    pt.containsKey("source") ? pt.get("source").toString() : "manual",
                    0.0,
                    0.0,
                    0.0
            ));
        }

        return new CalibrateV2.Phase2Data(
                raw.get("version").toString(),
                radarBlh,
                opticalBlh,
                raw.get("calibration_method").toString(),
                raw.get("calibrator").toString(),
                points
        );
    }

    private static StationBLH parseBLH(Map<String, Object> map) {
        return new StationBLH(
                toDouble(map.get("B")),
                toDouble(map.get("L")),
                toDouble(map.get("H"))
        );
    }

    private static double toDouble(Object obj) {
        if (obj instanceof Number n) return n.doubleValue();
        return Double.parseDouble(obj.toString());
    }

    private static void saveCalibrationV2(
            CalibrateV2.CalibrationResultV2 result,
            CalibrateV2.Phase2Data data,
            String outputPath) throws IOException {

        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        sb.append("  \"version\": \"").append(data.version()).append("\",\n");
        sb.append("  \"calibration_date\": \"").append(LocalDate.now()).append("\",\n");
        sb.append("  \"valid_until\": \"").append(LocalDate.now().plusDays(365)).append("\",\n");

        sb.append("  \"parameters\": {\n");
        sb.append("    \"radar_blh\": {\"B\": ").append(data.radarBlh().B())
                .append(", \"L\": ").append(data.radarBlh().L())
                .append(", \"H\": ").append(data.radarBlh().H()).append("},\n");
        sb.append("    \"optical_blh\": {\"B\": ").append(data.opticalBlh().B())
                .append(", \"L\": ").append(data.opticalBlh().L())
                .append(", \"H\": ").append(data.opticalBlh().H()).append("},\n");
        sb.append("    \"dAz0\": ").append(round6(result.dAz0())).append(",\n");
        sb.append("    \"dEl0\": ").append(round6(result.dEl0())).append(",\n");

        if (result.strategy() instanceof RadarErrorAnalyzer.FixedCompensation fc) {
            sb.append("    \"radar_compensation_type\": \"fixed\",\n");
            sb.append("    \"dAz_radar\": ").append(round6(fc.dAz())).append(",\n");
            sb.append("    \"dEl_radar\": ").append(round6(fc.dEl())).append(",\n");
        } else if (result.strategy() instanceof RadarErrorAnalyzer.SegmentedCompensation sc) {
            sb.append("    \"radar_compensation_type\": \"segmented\",\n");
            sb.append("    \"radar_compensation\": {\n");
            sb.append("      \"dimension\": \"").append(sc.dimension()).append("\",\n");
            sb.append("      \"segments\": [\n");

            int idx = 0;
            for (var entry : sc.segments().entrySet()) {
                var seg = entry.getValue();
                sb.append("        {\n");
                sb.append("          \"range\": \"").append(entry.getKey()).append("\",\n");
                sb.append("          \"min_km\": ").append(seg.config().min()).append(",\n");
                sb.append("          \"max_km\": ").append(
                        seg.config().max() == Double.MAX_VALUE ? "null" : seg.config().max()
                ).append(",\n");
                sb.append("          \"dAz\": ").append(round6(seg.meanAz())).append(",\n");
                sb.append("          \"dEl\": ").append(round6(seg.meanEl())).append(",\n");
                sb.append("          \"count\": ").append(seg.count()).append("\n");
                sb.append("        }");
                if (++idx < sc.segments().size()) sb.append(",");
                sb.append("\n");
            }
            sb.append("      ]\n");
            sb.append("    },\n");
        }
        sb.append("    \"radar2opticalTotalDelayMs\": 0\n");
        sb.append("  },\n");

        sb.append("  \"quality\": {\n");
        sb.append("    \"optical\": {\n");
        sb.append("      \"rms_az\": ").append(round6(result.rmsAz())).append(",\n");
        sb.append("      \"rms_el\": ").append(round6(result.rmsEl())).append(",\n");
        if (result.validCount() != result.totalCount()) {
            sb.append("      \"raw_rms_az\": ").append(round6(result.rawRmsAz())).append(",\n");
            sb.append("      \"raw_rms_el\": ").append(round6(result.rawRmsEl())).append(",\n");
        }
        sb.append("      \"valid_count\": ").append(result.validCount()).append(",\n");
        sb.append("      \"total_count\": ").append(result.totalCount()).append("\n");
        sb.append("    },\n");
        sb.append("    \"radar\": {\n");
        sb.append("      \"mean_az_error\": ").append(round6(result.radarAnalysis().overall().meanAz())).append(",\n");
        sb.append("      \"mean_el_error\": ").append(round6(result.radarAnalysis().overall().meanEl())).append(",\n");
        sb.append("      \"sample_count\": ").append(result.radarAnalysis().overall().count()).append("\n");
        sb.append("    }\n");
        sb.append("  },\n");

        sb.append("  \"radar_analysis\": {\n");
        var overall = result.radarAnalysis().overall();
        sb.append("    \"overall\": {\n");
        sb.append("      \"mean_az\": ").append(round6(overall.meanAz())).append(",\n");
        sb.append("      \"mean_el\": ").append(round6(overall.meanEl())).append(",\n");
        sb.append("      \"std_az\": ").append(round6(overall.stdAz())).append(",\n");
        sb.append("      \"std_el\": ").append(round6(overall.stdEl())).append(",\n");
        sb.append("      \"count\": ").append(overall.count()).append("\n");
        sb.append("    },\n");

        sb.append("    \"by_distance\": {\n");
        int di = 0;
        for (var entry : result.radarAnalysis().byDistance().entrySet()) {
            var seg = entry.getValue();
            sb.append("      \"").append(entry.getKey()).append("\": {\n");
            sb.append("        \"mean_az\": ").append(round6(seg.meanAz())).append(",\n");
            sb.append("        \"mean_el\": ").append(round6(seg.meanEl())).append(",\n");
            sb.append("        \"count\": ").append(seg.count()).append("\n");
            sb.append("      }");
            if (++di < result.radarAnalysis().byDistance().size()) sb.append(",");
            sb.append("\n");
        }
        sb.append("    },\n");

        sb.append("    \"recommendation\": \"").append(
                result.radarAnalysis().recommendation().replace("\n", "\\n")
                        .replace("\"", "\\\"")).append("\"\n");
        sb.append("  }\n");

        sb.append("}\n");

        Files.writeString(Path.of(outputPath), sb.toString());
        System.out.println("[OK] 保存到: " + outputPath);
    }

    private static void printReport(
            CalibrateV2.CalibrationResult optical,
            RadarErrorAnalyzer.RadarAnalysisResult radar) {

        System.out.println();
        System.out.println("============================================================");
        System.out.println("         Phase 2 标定报告（光电 + 雷达）");
        System.out.println("============================================================");

        System.out.println("\n[光电标定结果]");
        System.out.printf("  航向零点偏差  ΔAz0 = %+.6f°%n", optical.dAz0());
        System.out.printf("  俯仰零点偏差  ΔEl0 = %+.6f°%n", optical.dEl0());
        System.out.printf("  RMS(剔除后): Az=%.6f°, El=%.6f°%n", optical.rmsAz(), optical.rmsEl());
        if (optical.validCount() != optical.totalCount()) {
            System.out.printf("  RMS(原始):   Az=%.6f°, El=%.6f°%n", optical.rawRmsAz(), optical.rawRmsEl());
        }
        System.out.printf("  有效数据: %d/%d%n", optical.validCount(), optical.totalCount());

        System.out.println("\n[雷达误差分析]");
        var overall = radar.overall();
        System.out.printf("  方位偏差: %.4f° ± %.4f°%n", overall.meanAz(), overall.stdAz());
        System.out.printf("  俯仰偏差: %.4f° ± %.4f°%n", overall.meanEl(), overall.stdEl());
        System.out.printf("  样本数: %d%n", overall.count());

        System.out.println("\n[按距离分段]");
        radar.byDistance().forEach((k, v) ->
                System.out.printf("  %-8s: ΔAz=%.4f°, ΔEl=%.4f° (n=%d)%n", k, v.meanAz(), v.meanEl(), v.count()));

        System.out.println("\n[雷达补偿策略]");
        if (radar.strategy() instanceof RadarErrorAnalyzer.FixedCompensation fc) {
            System.out.println("  类型: 固定补偿");
            System.out.printf("  ΔAz_radar = %.6f°%n", fc.dAz());
            System.out.printf("  ΔEl_radar = %.6f°%n", fc.dEl());
        } else if (radar.strategy() instanceof RadarErrorAnalyzer.SegmentedCompensation sc) {
            System.out.printf("  类型: 分段补偿（按%s）%n", sc.dimension());
            sc.segments().forEach((k, v) ->
                    System.out.printf("  %-8s: ΔAz=%.6f°, ΔEl=%.6f°%n", k, v.meanAz(), v.meanEl()));
        }

        System.out.println("\n[结论]");
        System.out.println(radar.recommendation());
        System.out.println("============================================================");
    }

    private static double round6(double v) {
        return Math.round(v * 1_000_000.0) / 1_000_000.0;
    }

    // ==================== 数据模型 ====================

    public record Phase2Data(
            String version,
            StationBLH radarBlh,
            StationBLH opticalBlh,
            String calibrationMethod,
            String calibrator,
            List<CalibrateV2.SimulatedPoint> points
    ) {}

    /**
     * 标定点数据
     *
     * @param targetBlh      RTK真值目标坐标（必填）
     * @param radarBlh       雷达上报的目标坐标（可选）
     * @param azMeasured     光电方位角实测值
     * @param elMeasured     光电俯仰角实测值
     * @param timestamp      时间戳
     * @param targetId       目标ID
     * @param source         数据来源
     * @param radarAzError   已废弃，由程序自动计算
     * @param radarElError   已废弃，由程序自动计算
     * @param distanceKm     已废弃，由程序自动计算
     */
    public record SimulatedPoint(
            StationBLH targetBlh,
            StationBLH radarBlh,
            double azMeasured,
            double elMeasured,
            String timestamp,
            String targetId,
            String source,
            double radarAzError,
            double radarElError,
            double distanceKm
    ) {}

    public record CalibrationResult(
            double dAz0, double dEl0,
            double rmsAz, double rmsEl,
            double rawRmsAz, double rawRmsEl,
            int validCount, int totalCount
    ) {}

    public record CalibrationResultV2(
            double dAz0, double dEl0,
            double rmsAz, double rmsEl,
            double rawRmsAz, double rawRmsEl,
            int validCount, int totalCount,
            RadarErrorAnalyzer.CompensationStrategy strategy,
            RadarErrorAnalyzer.RadarAnalysisResult radarAnalysis
    ) {}
}
