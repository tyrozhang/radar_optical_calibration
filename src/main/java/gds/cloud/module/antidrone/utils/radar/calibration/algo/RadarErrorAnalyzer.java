package gds.cloud.module.antidrone.utils.radar.calibration.algo;

import gds.cloud.module.antidrone.utils.radar.calibration.model.SegmentConfig;

import java.util.*;

/**
 * 雷达误差分析器（Phase 2 核心）- 配置驱动版
 *
 * 修复：分段边界配置化，避免硬编码
 */
public class RadarErrorAnalyzer {

    // 默认分段配置（距离段）
    public static final List<SegmentConfig> DEFAULT_DISTANCE_SEGMENTS = List.of(
        new SegmentConfig("0-1km", 0.0, 1.0),
        new SegmentConfig("1-3km", 1.0, 3.0),
        new SegmentConfig("3-5km", 3.0, 5.0),
        new SegmentConfig("5-10km", 5.0, 10.0)
    );

    // 默认分段配置（高度段）
    public static final List<SegmentConfig> DEFAULT_HEIGHT_SEGMENTS = List.of(
        new SegmentConfig("0-100m", 0.0, 100.0),
        new SegmentConfig("100-200m", 100.0, 200.0),
        new SegmentConfig("200-500m", 200.0, 500.0),
        new SegmentConfig("500m+", 500.0, Double.MAX_VALUE)
    );

    // 固定阈值（兜底，仅在自适应不可用时使用）
    private double segmentThreshold = 0.03;  // 度

    // 自适应阈值开关（默认开启）
    private boolean adaptiveThreshold = true;

    // 自适应阈值范围限制（度）
    private static final double ADAPTIVE_MIN = 0.01;
    private static final double ADAPTIVE_MAX = 0.10;

    private final List<SegmentConfig> distanceSegments;
    private final List<SegmentConfig> heightSegments;

    public RadarErrorAnalyzer() {
        this(DEFAULT_DISTANCE_SEGMENTS, DEFAULT_HEIGHT_SEGMENTS);
    }

    public RadarErrorAnalyzer(List<SegmentConfig> distanceSegments, List<SegmentConfig> heightSegments) {
        this.distanceSegments = distanceSegments;
        this.heightSegments = heightSegments;
    }

    public void setSegmentThreshold(double threshold) {
        this.segmentThreshold = threshold;
    }

    public void setAdaptiveThreshold(boolean adaptive) {
        this.adaptiveThreshold = adaptive;
    }

    /**
     * 分析雷达误差
     */
    public RadarAnalysisResult analyze(List<ErrorSample> samples) {
        // 1. 计算总体统计
        OverallStats overall = computeOverallStats(samples);

        // 1.5 自适应阈值：基于设备精度
        double effectiveThreshold = this.segmentThreshold;
        if (adaptiveThreshold && overall.count() >= 5) {
            effectiveThreshold = computeAdaptiveThreshold(overall);
        }

        // 2. 按距离分段统计（配置驱动）
        Map<String, SegmentStats> byDistance = computeByDimension(samples, distanceSegments, Dimension.DISTANCE);

        // 3. 按高度分段统计（配置驱动）
        Map<String, SegmentStats> byHeight = computeByDimension(samples, heightSegments, Dimension.HEIGHT);

        // 4. 决策：固定补偿 vs 分段补偿（使用有效阈值）
        CompensationStrategy strategy = decideStrategy(overall, byDistance, byHeight, effectiveThreshold);

        // 5. 生成推荐
        String recommendation = generateRecommendation(overall, byDistance, byHeight, strategy, effectiveThreshold);

        return new RadarAnalysisResult(
            overall,
            byDistance,
            byHeight,
            strategy,
            recommendation
        );
    }

    /**
     * 自适应计算分段决策阈值
     *
     * 公式：threshold = max(overall_std_Az, overall_std_El)
     *
     * 物理含义：如果各段均值差异不超过总体标准差，
     * 说明"段间差异"被"段内离散"淹没，分段无统计意义。
     *
     * 兜底保护：不低于 ADAPTIVE_MIN（避免高精度设备过度分段），
     *           不高于 ADAPTIVE_MAX（避免低精度设备永不分段）。
     */
    private double computeAdaptiveThreshold(OverallStats overall) {
        double threshold = Math.max(overall.stdAz(), overall.stdEl());
        return Math.max(ADAPTIVE_MIN, Math.min(ADAPTIVE_MAX, threshold));
    }

    /**
     * 计算总体统计
     */
    private OverallStats computeOverallStats(List<ErrorSample> samples) {
        int n = samples.size();
        if (n == 0) return new OverallStats(0, 0, 0, 0, 0);

        double sumAz = samples.stream().mapToDouble(ErrorSample::azError).sum();
        double sumEl = samples.stream().mapToDouble(ErrorSample::elError).sum();

        double meanAz = sumAz / n;
        double meanEl = sumEl / n;

        double varAz = samples.stream()
            .mapToDouble(s -> Math.pow(s.azError() - meanAz, 2)).sum() / n;
        double varEl = samples.stream()
            .mapToDouble(s -> Math.pow(s.elError() - meanEl, 2)).sum() / n;

        return new OverallStats(meanAz, meanEl, Math.sqrt(varAz), Math.sqrt(varEl), n);
    }

    /**
     * 按维度分段统计（配置驱动）
     */
    private Map<String, SegmentStats> computeByDimension(
            List<ErrorSample> samples,
            List<SegmentConfig> segments,
            Dimension dim) {

        Map<String, List<ErrorSample>> grouped = new LinkedHashMap<>();

        // 初始化分组
        for (SegmentConfig seg : segments) {
            grouped.put(seg.name(), new ArrayList<>());
        }

        // 分组
        for (ErrorSample s : samples) {
            double value = dim == Dimension.DISTANCE ? s.distanceKm() : s.heightM();
            for (SegmentConfig seg : segments) {
                if (seg.contains(value)) {
                    grouped.get(seg.name()).add(s);
                    break;
                }
            }
        }

        // 计算统计
        Map<String, SegmentStats> result = new LinkedHashMap<>();
        for (SegmentConfig seg : segments) {
            List<ErrorSample> group = grouped.get(seg.name());
            if (!group.isEmpty()) {
                result.put(seg.name(), computeStats(group, seg));
            }
        }

        return result;
    }

    private SegmentStats computeStats(List<ErrorSample> samples, SegmentConfig config) {
        int n = samples.size();
        double meanAz = samples.stream().mapToDouble(ErrorSample::azError).sum() / n;
        double meanEl = samples.stream().mapToDouble(ErrorSample::elError).sum() / n;
        double varAz = samples.stream()
            .mapToDouble(s -> Math.pow(s.azError() - meanAz, 2)).sum() / n;
        double varEl = samples.stream()
            .mapToDouble(s -> Math.pow(s.elError() - meanEl, 2)).sum() / n;
        double meanHeight = samples.stream().mapToDouble(ErrorSample::heightM).sum() / n;
        double meanDist = samples.stream().mapToDouble(ErrorSample::distanceKm).sum() / n;

        return new SegmentStats(
            config,
            meanAz, meanEl,
            Math.sqrt(varAz), Math.sqrt(varEl),
            meanHeight, meanDist, n
        );
    }

    /**
     * 决策：选择补偿策略
     *
     * 统一使用距离维度判断，理由：
     * 1. 雷达测角误差通常与距离相关性更强
     * 2. 方位和俯仰使用同一维度，逻辑一致
     * 3. 高度分段仅用于辅助分析报告
     */
    private CompensationStrategy decideStrategy(
            OverallStats overall,
            Map<String, SegmentStats> byDistance,
            Map<String, SegmentStats> byHeight,
            double effectiveThreshold) {

        // 统一使用距离分段判断：方位和俯仰误差各自与总体值比较
        double maxAzDiff = byDistance.values().stream()
            .mapToDouble(s -> Math.abs(s.meanAz() - overall.meanAz()))
            .max().orElse(0.0);

        double maxElDiff = byDistance.values().stream()
            .mapToDouble(s -> Math.abs(s.meanEl() - overall.meanEl()))
            .max().orElse(0.0);

        if (maxAzDiff > effectiveThreshold || maxElDiff > effectiveThreshold) {
            // 误差变化显著，使用距离分段补偿
            return new SegmentedCompensation("distance", byDistance);
        } else {
            // 误差变化不显著，使用固定补偿
            return new FixedCompensation(overall.meanAz(), overall.meanEl());
        }
    }

    /**
     * 生成分析报告
     */
    private String generateRecommendation(
            OverallStats overall,
            Map<String, SegmentStats> byDistance,
            Map<String, SegmentStats> byHeight,
            CompensationStrategy strategy,
            double effectiveThreshold) {

        StringBuilder sb = new StringBuilder();
        sb.append("雷达误差分析结论：\n");
        sb.append(String.format("1. 总体偏差：方位 %.4f° ± %.4f°，俯仰 %.4f° ± %.4f°（样本数 %d）\n",
            overall.meanAz(), overall.stdAz(), overall.meanEl(), overall.stdEl(), overall.count()));

        if (strategy instanceof SegmentedCompensation s) {
            sb.append(String.format("2. 误差随距离变化明显（决策阈值 %.4f°），建议使用分段补偿\n", effectiveThreshold));
            sb.append("3. 各段补偿值：\n");
            s.segments().forEach((k, v) ->
                sb.append(String.format("   %s: ΔAz=%.4f°, ΔEl=%.4f° (n=%d)\n",
                    k, v.meanAz(), v.meanEl(), v.count())));
        } else {
            sb.append(String.format("2. 误差变化不显著（决策阈值 %.4f°），使用固定补偿即可\n", effectiveThreshold));
        }

        return sb.toString();
    }

    // ==================== 数据结构和配置 ====================

    public record ErrorSample(double distanceKm, double heightM, double azError, double elError) {}

    public record OverallStats(double meanAz, double meanEl, double stdAz, double stdEl, int count) {}

    public record SegmentStats(
        SegmentConfig config,
        double meanAz, double meanEl,
        double stdAz, double stdEl,
        double meanHeight, double meanDistance, int count
    ) {}

    public sealed interface CompensationStrategy
        permits FixedCompensation, SegmentedCompensation {}

    public record FixedCompensation(double dAz, double dEl) implements CompensationStrategy {}

    public record SegmentedCompensation(String dimension, Map<String, SegmentStats> segments)
        implements CompensationStrategy {}

    public record RadarAnalysisResult(
        OverallStats overall,
        Map<String, SegmentStats> byDistance,
        Map<String, SegmentStats> byHeight,
        CompensationStrategy strategy,
        String recommendation
    ) {}

    private enum Dimension { DISTANCE, HEIGHT }
}
