package gds.cloud.module.antidrone.utils.radar.calibration.algo;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;
import gds.cloud.module.antidrone.utils.radar.calibration.util.SimpleJsonParser;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.format.DateTimeFormatter;
import java.util.*;

/**
 * Phase 2 模拟数据生成器（修复版）
 *
 * 关键修复：
 * 1. 角度误差与坐标偏移物理自洽 —— 角度误差驱动坐标计算
 * 2. 样本量增至30个，均匀覆盖4个距离段
 * 3. 距离相关误差模型更符合雷达物理特性
 */
public class Simulator {

    // 雷达误差模型参数（可配置）
    private double azMeanError = 0.05;      // 方位平均误差（度）
    private double elMeanError = 0.02;      // 俯仰平均误差（度）；俯仰约定：向上为负/向下为正，
                                            // 正值表示雷达上报位置偏向地面方向（俯视偏差）
    private double azStdError = 0.015;      // 方位标准差
    private double elStdError = 0.008;      // 俯仰标准差
    private double distanceFactor = 0.01;   // 距离相关误差系数

    // 样本配置
    private static final int SAMPLES_PER_SEGMENT = 8;  // 每段8个样本
    private static final double[][] DISTANCE_SEGMENTS = {
        {0.0, 1.0},    // 0-1km
        {1.0, 3.0},    // 1-3km
        {3.0, 5.0},    // 3-5km
        {5.0, 10.0}    // 5-10km（扩展上限，避免5km+过于稀疏）
    };

    private StationBLH opticalBlh;
    private StationBLH radarBlh;  // 雷达站坐标（用于从雷达站视角生成误差）
    private Random rand;

    public Simulator() {
        this.rand = new Random(42); // 固定种子，保证可复现
    }

    public Simulator(long seed) {
        this.rand = new Random(seed);
    }

    /**
     * 生成 Phase 2 格式的模拟数据（30个样本，均匀分布）
     */
    public CalibrationDataV2 generate(String inputPath) throws IOException {
        // 1. 读取原始数据获取基准参数
        OriginalData original = readOriginalData(inputPath);
        this.opticalBlh = original.opticalBlh();
        this.radarBlh = original.radarBlh();

        System.out.println("[*] 生成模拟数据...");
        System.out.println("    光电站: B=" + opticalBlh.B() + ", L=" + opticalBlh.L() + ", H=" + opticalBlh.H());
        System.out.println("    雷达站: B=" + radarBlh.B() + ", L=" + radarBlh.L() + ", H=" + radarBlh.H());

        // 2. 生成30个均匀分布的样本
        List<SimulatedPoint> points = new ArrayList<>();
        int targetId = 1;

        for (double[] segment : DISTANCE_SEGMENTS) {
            double minDist = segment[0];
            double maxDist = segment[1];

            for (int i = 0; i < SAMPLES_PER_SEGMENT; i++) {
                // 在段内均匀分布距离
                double distKm = minDist + (maxDist - minDist) * (i + 0.5) / SAMPLES_PER_SEGMENT;

                // 随机方位角（0-360度，均匀分布）
                double azimuth = rand.nextDouble() * 360.0;

                // 随机高度（50-500米）
                double height = 50 + rand.nextDouble() * 450;

                SimulatedPoint point = generatePoint(distKm, azimuth, height, targetId++);
                points.add(point);
            }
        }

        // 3. 构建结果
        return new CalibrationDataV2(
            "2.0",
            original.radarBlh(),
            original.opticalBlh(),
            original.calibrationMethod(),
            original.calibrator(),
            points,
            new SimulatorConfig(azMeanError, elMeanError, azStdError, elStdError, distanceFactor)
        );
    }

    /**
     * 生成单个模拟点 —— 从雷达站视角生成角度误差
     *
     * 物理一致性保证（与 CalibrateV2 计算逻辑自洽）：
     * 1. 计算从雷达站到真实目标的几何角（雷达"应该"指的角度）
     * 2. 生成距离相关的角度误差（雷达测角误差模型）
     * 3. 雷达观测角 = 几何角 + 误差 → 沿此方向计算 radar_blh
     * 4. 光电实测角 = 光电站几何角 + 光电噪声
     * 5. CalibrateV2 侧：azError = Az_measured - radarGeoAz，可反推出注入的误差
     */
    private SimulatedPoint generatePoint(double distKm, double azimuth, double height, int targetId) {
        // 1. 计算真实目标位置（RTK真值）—— 以光电站为参考
        StationBLH targetBlh = calculateTargetPosition(distKm, azimuth, height);

        // 2. 计算从雷达站到真实目标的几何角（雷达"应该"指的角度）
        double[] radarGeoAzEl = CoordinateUtils.blhToAzEl(
            targetBlh.B(), targetBlh.L(), targetBlh.H(),
            radarBlh.B(), radarBlh.L(), radarBlh.H()
        );
        double radarTrueAz = radarGeoAzEl[0];

        // 3. 生成距离相关的角度误差（雷达测角误差模型）
        double distFactor = 1.0 + distanceFactor * distKm;
        double azError = gaussianRandom(azMeanError * distFactor, azStdError * distFactor);
        double elError = gaussianRandom(elMeanError * distFactor, elStdError * distFactor);

        // 4. 从雷达站沿(radarObsAz, radarObsEl)方向计算雷达上报的目标BLH
        //    精确三角法：(R, Az+δAz, El+δEl) 直接重建笛卡尔坐标
        double slantRange = CoordinateUtils.distance3D(radarBlh, targetBlh) / 1000.0;  // km
        double deltaH = (targetBlh.H() - radarBlh.H()) / 1000.0;                     // km
        double dHoriz = Math.sqrt(Math.max(0, slantRange * slantRange - deltaH * deltaH));
        double radarTrueElGeom = Math.toDegrees(Math.atan2(deltaH, dHoriz));          // 几何俯仰角
        StationBLH radarReportedBlh = calculatePositionFromAnglesWithError(
            radarBlh, radarTrueAz, radarTrueElGeom, slantRange, azError, elError
        );

        // 9. 计算光电站到真实目标的几何角，生成光电实测角度
        double[] opticalGeoAzEl = CoordinateUtils.blhToAzEl(
            targetBlh.B(), targetBlh.L(), targetBlh.H(),
            opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
        );
        double opticAzNoise = gaussianRandom(0, 0.02);  // 光电测量噪声
        double opticElNoise = gaussianRandom(0, 0.01);
        double azMeasured = opticalGeoAzEl[0] + opticAzNoise;
        double elMeasured = opticalGeoAzEl[1] + opticElNoise;

        // 8. 计算雷达站到目标的真实 3D 斜距（用于标定输出）
        double realDistKm = CoordinateUtils.distance3D(radarBlh, targetBlh) / 1000.0;

        return new SimulatedPoint(
            targetBlh,          // RTK真值
            radarReportedBlh,   // 雷达上报（从雷达站沿观测角方向计算）
            azMeasured,         // 光电实测
            elMeasured,
            Instant.now().toString(),
            "SIM_" + String.format("%04d", targetId),
            "simulated",
            azError,            // 雷达方位误差（注入值，用于验证）
            elError,            // 雷达俯仰误差（注入值，用于验证）
            realDistKm          // 雷达站到目标的真实 3D 斜距
        );
    }

    /**
     * 根据水平距离、方位角、高度计算目标BLH（haversine 球面公式）
     *
     * Haversine 在小距离时精度优于 ENU 展开（尤其对有高度差的目标），
     * 3D 距离误差 < 0.15%，满足 Phase 2 模拟精度要求。
     */
    private StationBLH calculateTargetPosition(double distKm, double azimuth, double height) {
        double distRad = distKm / 6371.0;  // 地球平均半径
        double azRad = Math.toRadians(azimuth);

        double lat1 = Math.toRadians(opticalBlh.B());
        double lon1 = Math.toRadians(opticalBlh.L());

        // 球面三角
        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(distRad) +
                                Math.cos(lat1) * Math.sin(distRad) * Math.cos(azRad));
        double lon2 = lon1 + Math.atan2(Math.sin(azRad) * Math.sin(distRad) * Math.cos(lat1),
                                        Math.cos(distRad) - Math.sin(lat1) * Math.sin(lat2));

        return new StationBLH(Math.toDegrees(lat2), Math.toDegrees(lon2), height);
    }

    /**
     * 带角度误差的目标位置计算（精确三角法）
     *
     * 物理一致性保证：
     * 1. 从几何角(真斜距+几何俯仰角)推导笛卡尔坐标（精确，无近似）
     * 2. 用精确三角公式施加角度误差，再重建笛卡尔坐标
     * 3. 当 CalibrateV2 反算时，角度误差被精确还原
     *
     * @param station      雷达站 BLH
     * @param azTrue       真值方位角（North=0, 顺时针）
     * @param elGeom       几何俯仰角（度，水平=0）
     * @param slantRangeKm 真实 3D 斜距（km）
     * @param azError      方位角误差（度）
     * @param elError      俯仰角误差（度）
     */
    private StationBLH calculatePositionFromAnglesWithError(
            StationBLH station, double azTrue, double elGeom,
            double slantRangeKm, double azError, double elError) {

        double R = slantRangeKm * 1000;                    // 斜距 (m)

        // 雷达观测角 = 真值 + 误差（直接加法，无近似）
        double azObs = azTrue + azError;
        double elObs = elGeom + elError;

        // 从 (R, Az+δAz, El+δEl) 用球坐标精确重建 ENU
        double azObsRad = Math.toRadians(azObs);
        double elObsRad = Math.toRadians(elObs);

        double e = R * Math.cos(elObsRad) * Math.sin(azObsRad);
        double n = R * Math.cos(elObsRad) * Math.cos(azObsRad);
        double u = R * Math.sin(elObsRad);

        return blhFromEnu(station, e, n, u);
    }

    /**
     * ENU→BLH（精确 Bowring 迭代法）
     *
     * 先 ENU→ECEF，再 ECEF→BLH（Bowring 迭代法）。
     * Bowring 公式中的分母项 (1-E2*sin²θ)^(3/2) 在高仰角时不可省略，
     * 否则高度计算偏差可达数十米（对于近距高仰角目标）。
     *
     * Bowring 迭代公式：
     *   Numerator:   Z + e2*(1-e2)*A*sin³(θ) / (1-e2*sin²(θ))^(3/2)
     *   Denominator: p - e2*A*cos³(θ)      / (1-e2*sin²(θ))^(3/2)
     */
    private StationBLH blhFromEnu(StationBLH station, double e, double n, double u) {
        // 1. ENU → ECEF
        double[] ecefS = CoordinateUtils.blhToEcef(station.B(), station.L(), station.H());
        double latRad = Math.toRadians(station.B()), lonRad = Math.toRadians(station.L());
        double sinB = Math.sin(latRad), cosB = Math.cos(latRad);
        double sinL = Math.sin(lonRad), cosL = Math.cos(lonRad);

        double X = ecefS[0] - sinL * e - sinB * cosL * n + cosB * cosL * u;
        double Y = ecefS[1] + cosL * e - sinB * sinL * n + cosB * sinL * u;
        double Z = ecefS[2] + cosB * n + sinB * u;

        // 2. ECEF → BLH（Bowring 迭代）
        double p = Math.sqrt(X * X + Y * Y);
        double theta = Math.atan2(Z * CoordinateUtils.A, p * (1 - CoordinateUtils.E2) * CoordinateUtils.A);

        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);
        double sinTheta2 = sinTheta * sinTheta;
        double cosTheta3 = cosTheta * cosTheta * cosTheta;
        double denom = Math.pow(1.0 - CoordinateUtils.E2 * sinTheta2, 1.5);

        double B_rad = Math.atan2(
            Z + CoordinateUtils.E2 * (1.0 - CoordinateUtils.E2) * CoordinateUtils.A * sinTheta2 * sinTheta / denom,
            p - CoordinateUtils.E2 * CoordinateUtils.A * cosTheta3 / denom
        );

        double L_rad = Math.atan2(Y, X);
        double sinB2 = Math.sin(B_rad) * Math.sin(B_rad);
        double N = CoordinateUtils.A / Math.sqrt(1.0 - CoordinateUtils.E2 * sinB2);
        double H = p / Math.cos(B_rad) - N;

        return new StationBLH(Math.toDegrees(B_rad), Math.toDegrees(L_rad), H);
    }

    private double gaussianRandom(double mean, double std) {
        return mean + std * rand.nextGaussian();
    }

    private OriginalData readOriginalData(String path) throws IOException {
        String json = Files.readString(Path.of(path));
        Map<String, Object> raw = SimpleJsonParser.parse(json);

        @SuppressWarnings("unchecked")
        Map<String, Object> radarMap = (Map<String, Object>) raw.get("radar_blh");
        @SuppressWarnings("unchecked")
        Map<String, Object> opticalMap = (Map<String, Object>) raw.get("optical_blh");

        StationBLH radarBlh = new StationBLH(
            toDouble(radarMap.get("B")),
            toDouble(radarMap.get("L")),
            toDouble(radarMap.get("H"))
        );
        StationBLH opticalBlh = new StationBLH(
            toDouble(opticalMap.get("B")),
            toDouble(opticalMap.get("L")),
            toDouble(opticalMap.get("H"))
        );

        String method = raw.containsKey("calibration_method")
            ? raw.get("calibration_method").toString() : "RTK无人机";
        String calibrator = raw.containsKey("calibrator")
            ? raw.get("calibrator").toString() : "系统模拟";

        return new OriginalData(radarBlh, opticalBlh, method, calibrator, List.of());
    }

    private double toDouble(Object obj) {
        if (obj instanceof Number n) return n.doubleValue();
        return Double.parseDouble(obj.toString());
    }

    /**
     * 保存为 JSON 文件
     */
    public void saveToJson(CalibrationDataV2 data, String outputPath) throws IOException {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        sb.append("  \"version\": \"").append(data.version()).append("\",\n");
        sb.append("  \"radar_blh\": {\"B\": ").append(data.radarBlh().B())
            .append(", \"L\": ").append(data.radarBlh().L())
            .append(", \"H\": ").append(data.radarBlh().H()).append("},\n");
        sb.append("  \"optical_blh\": {\"B\": ").append(data.opticalBlh().B())
            .append(", \"L\": ").append(data.opticalBlh().L())
            .append(", \"H\": ").append(data.opticalBlh().H()).append("},\n");
        sb.append("  \"calibration_method\": \"").append(data.calibrationMethod()).append("\",\n");
        sb.append("  \"calibrator\": \"").append(data.calibrator()).append("\",\n");
        sb.append("  \"data_points\": [\n");

        for (int i = 0; i < data.dataPoints().size(); i++) {
            SimulatedPoint p = data.dataPoints().get(i);
            sb.append("    {\n");
            sb.append("      \"target_blh\": {\"B\": ").append(p.targetBlh().B())
                .append(", \"L\": ").append(p.targetBlh().L())
                .append(", \"H\": ").append(p.targetBlh().H()).append("},\n");
            sb.append("      \"radar_blh\": {\"B\": ").append(p.radarBlh().B())
                .append(", \"L\": ").append(p.radarBlh().L())
                .append(", \"H\": ").append(p.radarBlh().H()).append("},\n");
            sb.append("      \"Az_measured\": ").append(p.azMeasured()).append(",\n");
            sb.append("      \"El_measured\": ").append(p.elMeasured()).append(",\n");
            sb.append("      \"timestamp\": \"").append(p.timestamp()).append("\",\n");
            sb.append("      \"target_id\": \"").append(p.targetId()).append("\",\n");
            sb.append("      \"source\": \"").append(p.source()).append("\",\n");
            sb.append("      \"radar_az_error\": ").append(String.format("%.6f", p.radarAzError())).append(",\n");
            sb.append("      \"radar_el_error\": ").append(String.format("%.6f", p.radarElError())).append(",\n");
            sb.append("      \"distance_km\": ").append(String.format("%.3f", p.distanceKm())).append("\n");
            sb.append("    }").append(i < data.dataPoints().size() - 1 ? "," : "").append("\n");
        }

        sb.append("  ],\n");
        sb.append("  \"simulator_config\": {\n");
        sb.append("    \"az_mean_error\": ").append(data.simulatorConfig().azMeanError()).append(",\n");
        sb.append("    \"el_mean_error\": ").append(data.simulatorConfig().elMeanError()).append(",\n");
        sb.append("    \"az_std_error\": ").append(data.simulatorConfig().azStdError()).append(",\n");
        sb.append("    \"el_std_error\": ").append(data.simulatorConfig().elStdError()).append(",\n");
        sb.append("    \"distance_factor\": ").append(data.simulatorConfig().distanceFactor()).append("\n");
        sb.append("  }\n");
        sb.append("}\n");

        Files.writeString(Path.of(outputPath), sb.toString());
        System.out.println("[OK] 模拟数据已保存到: " + outputPath);
        System.out.println("    共 " + data.dataPoints().size() + " 个数据点");

        // 打印分段统计
        printSegmentStats(data.dataPoints());
    }

    private void printSegmentStats(List<SimulatedPoint> points) {
        System.out.println("\n[*] 分段分布统计:");
        Map<String, Integer> counts = new LinkedHashMap<>();
        counts.put("0-1km", 0);
        counts.put("1-3km", 0);
        counts.put("3-5km", 0);
        counts.put("5-10km", 0);

        for (SimulatedPoint p : points) {
            double d = p.distanceKm();
            if (d < 1) counts.put("0-1km", counts.get("0-1km") + 1);
            else if (d < 3) counts.put("1-3km", counts.get("1-3km") + 1);
            else if (d < 5) counts.put("3-5km", counts.get("3-5km") + 1);
            else counts.put("5-10km", counts.get("5-10km") + 1);
        }

        counts.forEach((k, v) -> System.out.println("    " + k + ": " + v + " 个样本"));
    }

    // ==================== 数据模型 ====================

    public record OriginalData(
        StationBLH radarBlh,
        StationBLH opticalBlh,
        String calibrationMethod,
        String calibrator,
        List<OriginalPoint> dataPoints
    ) {}

    public record OriginalPoint(StationBLH targetBlh, double azMeasured, double elMeasured) {}

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

    public record CalibrationDataV2(
        String version,
        StationBLH radarBlh,
        StationBLH opticalBlh,
        String calibrationMethod,
        String calibrator,
        List<SimulatedPoint> dataPoints,
        SimulatorConfig simulatorConfig
    ) {}

    public record SimulatorConfig(
        double azMeanError,
        double elMeanError,
        double azStdError,
        double elStdError,
        double distanceFactor
    ) {}

    // ==================== 主程序 ====================

    public static void main(String[] args) throws IOException {
        String inputPath = "calibration_data.json";
        String outputPath = "calibration_data_v2.json";

        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-i") && i + 1 < args.length) inputPath = args[++i];
            if (args[i].equals("-o") && i + 1 < args.length) outputPath = args[++i];
        }

        System.out.println("[*] Phase 2 模拟数据生成器（修复版）");
        System.out.println("    输入: " + inputPath);
        System.out.println("    输出: " + outputPath);
        System.out.println("    样本数: 32个（每段8个 × 4段）");

        Simulator simulator = new Simulator();
        CalibrationDataV2 data = simulator.generate(inputPath);
        simulator.saveToJson(data, outputPath);
    }
}
