package gds.cloud.module.antidrone.utils.radar.calibration.algo;

import gds.cloud.module.antidrone.utils.radar.calibration.util.SimpleJsonParser;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.util.*;
import java.util.logging.Logger;

/**
 * 焦距标定处理器
 *
 * 职责：
 * - 读取焦距标定输入数据（focal_calibration_data.json）
 * - 校验数据（距离正序、焦距范围、野值剔除）
 * - 生成焦距查表条目
 * - 保存结果：
 *   - 独立输出（focal_length_table.json）
 *   - 合并到现有 calibration_v2.json
 *
 * 与角度标定（CalibrateV2）的关系：
 * - 标定过程独立（不需要RTK真值，不需要雷达数据）
 * - 配置存储合并（最终写入同一个 calibration_v2.json）
 *
 * @see CalibrateV2
 */
public class FocalCalibrator {

    private static final Logger log = Logger.getLogger(FocalCalibrator.class.getName());

    // ==================== 数据模型 ====================

    /**
     * 焦距标定输入数据
     *
     * @param version       格式版本
     * @param calibrationDate 标定日期
     * @param device        光电设备信息（可为 null）
     * @param calibrator    标定人员
     * @param method        标定方法
     * @param points        标定数据点
     */
    public record FocalCalibrationData(
            String version,
            String calibrationDate,
            OpticalDevice device,
            String calibrator,
            String method,
            List<FocalCalibrationPoint> points
    ) {}

    /**
     * 光电设备信息
     *
     * @param model            设备型号
     * @param focalLengthMinMm 最小焦距/广角端（mm），可选，用于数据校验
     * @param focalLengthMaxMm 最大焦距/望远端（mm），可选，用于数据校验
     */
    public record OpticalDevice(
            String model,
            Double focalLengthMinMm,
            Double focalLengthMaxMm
    ) {}

    /**
     * 焦距标定数据点
     *
     * @param distanceM     目标距离（米），必须 > 0
     * @param focalLengthMm 清晰时的焦距（mm），必须 > 0
     */
    public record FocalCalibrationPoint(
            double distanceM,
            double focalLengthMm
    ) {}

    /**
     * 焦距标定结果
     *
     * @param validPoints  校验通过的数据点（按距离升序）
     * @param removedCount 被野值剔除的点数
     * @param device       设备信息
     */
    public record FocalCalibrationResult(
            List<FocalCalibrationPoint> validPoints,
            int removedCount,
            OpticalDevice device
    ) {}

    // ==================== 主入口 ====================

    public static void main(String[] args) throws IOException {
        String inputPath = "focal_calibration_data.json";
        String standaloneOutputPath = "focal_length_table_temp.json";
        String calibrationV2Path = "calibration_v2.json";

        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-i") && i + 1 < args.length) inputPath = args[++i];
            if (args[i].equals("-o") && i + 1 < args.length) standaloneOutputPath = args[++i];
            if (args[i].equals("--merge") && i + 1 < args.length) calibrationV2Path = args[++i];
            if (args[i].equals("--standalone")) {
                processStandalone(inputPath, standaloneOutputPath);
                return;
            }
            if (args[i].equals("--merge-only")) {
                processMerge(inputPath, calibrationV2Path);
                return;
            }
        }

        // 默认：两种输出都执行
        FocalCalibrationResult result = process(inputPath);
        saveStandalone(result, standaloneOutputPath);
        log.info("独立输出已保存: " + standaloneOutputPath);

        if (Files.exists(Path.of(calibrationV2Path))) {
            mergeToCalibrationV2(result, calibrationV2Path);
            log.info("已合并到: " + calibrationV2Path);
        } else {
            log.warning("calibration_v2.json 不存在，跳过合并: " + calibrationV2Path);
        }
    }

    // ==================== 公开 API ====================

    /**
     * 处理焦距标定数据（全流程：读取 → 校验 → 野值剔除）
     *
     * @param inputPath 标定输入文件路径
     * @return 标定结果
     */
    public static FocalCalibrationResult process(String inputPath) throws IOException {
        log.info("焦距标定处理");
        log.info("输入: " + inputPath);

        // 1. 读取输入数据
        FocalCalibrationData data = loadFocalCalibrationData(inputPath);
        log.info("共 " + data.points().size() + " 个数据点");

        // 2. 基础校验
        validateBasic(data);

        // 3. 排序（按距离升序）
        List<FocalCalibrationPoint> sorted = new ArrayList<>(data.points());
        sorted.sort(Comparator.comparingDouble(FocalCalibrationPoint::distanceM));

        // 4. 检查距离重复
        checkDuplicateDistances(sorted);

        // 5. 3σ 野值剔除（焦距维度）
        FocalCalibrationResult result = filterOutliers(sorted, data.device());

        log.info(String.format("标定完成: 有效 %d 点, 剔除 %d 点",
                result.validPoints().size(), result.removedCount()));

        return result;
    }

    /**
     * 处理并保存为独立输出文件
     */
    public static void processStandalone(String inputPath, String outputPath) throws IOException {
        FocalCalibrationResult result = process(inputPath);
        saveStandalone(result, outputPath);
        log.info("独立输出已保存: " + outputPath);
    }

    /**
     * 处理并合并到现有 calibration_v2.json
     */
    public static void processMerge(String inputPath, String calibrationV2Path) throws IOException {
        FocalCalibrationResult result = process(inputPath);
        mergeToCalibrationV2(result, calibrationV2Path);
        log.info("已合并到: " + calibrationV2Path);
    }

    // ==================== 数据加载 ====================

    /**
     * 读取焦距标定输入文件
     */
    static FocalCalibrationData loadFocalCalibrationData(String path) throws IOException {
        String json = Files.readString(Path.of(path));
        Map<String, Object> raw = SimpleJsonParser.parse(json);

        String version = raw.get("version") != null ? raw.get("version").toString() : "1.0";
        String calibrationDate = raw.get("calibration_date") != null
                ? raw.get("calibration_date").toString() : LocalDate.now().toString();
        String calibrator = raw.get("calibrator") != null ? raw.get("calibrator").toString() : "";
        String method = raw.get("calibration_method") != null ? raw.get("calibration_method").toString() : "";

        // 解析设备信息（可选）
        OpticalDevice device = null;
        if (raw.get("optical_device") != null) {
            @SuppressWarnings("unchecked")
            Map<String, Object> deviceMap = (Map<String, Object>) raw.get("optical_device");
            device = new OpticalDevice(
                    deviceMap.get("model") != null ? deviceMap.get("model").toString() : null,
                    deviceMap.get("focal_length_min_mm") != null ? toDouble(deviceMap.get("focal_length_min_mm")) : null,
                    deviceMap.get("focal_length_max_mm") != null ? toDouble(deviceMap.get("focal_length_max_mm")) : null
            );
        }

        // 解析数据点
        @SuppressWarnings("unchecked")
        List<Object> pointsRaw = (List<Object>) raw.get("data_points");
        if (pointsRaw == null || pointsRaw.isEmpty()) {
            throw new IllegalArgumentException("焦距标定数据为空：data_points 缺失或为空");
        }

        List<FocalCalibrationPoint> points = new ArrayList<>();
        for (Object obj : pointsRaw) {
            @SuppressWarnings("unchecked")
            Map<String, Object> pt = (Map<String, Object>) obj;

            double distanceM = toDouble(pt.get("distance_m"));
            double focalLengthMm = toDouble(pt.get("focal_length_mm"));

            points.add(new FocalCalibrationPoint(distanceM, focalLengthMm));
        }

        return new FocalCalibrationData(version, calibrationDate, device, calibrator, method, points);
    }

    // ==================== 校验 ====================

    /**
     * 基础校验
     */
    private static void validateBasic(FocalCalibrationData data) {
        List<String> errors = new ArrayList<>();

        for (int i = 0; i < data.points().size(); i++) {
            FocalCalibrationPoint p = data.points().get(i);
            if (p.distanceM() <= 0) {
                errors.add(String.format("数据点 %d: 距离必须 > 0，当前 %.2f", i, p.distanceM()));
            }
            if (p.focalLengthMm() <= 0) {
                errors.add(String.format("数据点 %d: 焦距必须 > 0，当前 %.2f", i, p.focalLengthMm()));
            }
            // 焦距范围校验（如果设备信息提供了范围）
            if (data.device() != null && data.device().focalLengthMinMm() != null) {
                if (p.focalLengthMm() < data.device().focalLengthMinMm()) {
                    errors.add(String.format("数据点 %d: 焦距 %.2fmm 低于设备最小值 %.2fmm",
                            i, p.focalLengthMm(), data.device().focalLengthMinMm()));
                }
            }
            if (data.device() != null && data.device().focalLengthMaxMm() != null) {
                if (p.focalLengthMm() > data.device().focalLengthMaxMm()) {
                    errors.add(String.format("数据点 %d: 焦距 %.2fmm 超过设备最大值 %.2fmm",
                            i, p.focalLengthMm(), data.device().focalLengthMaxMm()));
                }
            }
        }

        if (data.points().size() < 2) {
            errors.add("至少需要 2 个数据点才能构建焦距查表");
        }

        if (!errors.isEmpty()) {
            String msg = "焦距标定数据校验失败:\n" + String.join("\n", errors);
            log.severe(msg);
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * 检查距离重复（允许非常接近的距离，但完全相同则报错）
     */
    private static void checkDuplicateDistances(List<FocalCalibrationPoint> sorted) {
        for (int i = 1; i < sorted.size(); i++) {
            double prev = sorted.get(i - 1).distanceM();
            double curr = sorted.get(i).distanceM();
            // 距离差小于 1 米视为重复
            if (Math.abs(curr - prev) < 1.0) {
                throw new IllegalArgumentException(
                        String.format("距离重复或过于接近: %.1fm 和 %.1fm（差值 < 1m），请合并或删除其一", prev, curr));
            }
        }
    }

    // ==================== 野值剔除 ====================

    /**
     * 3σ 野值剔除（焦距维度）
     *
     * 剔除条件：|focalLength - 预期焦距| > 3σ
     * 预期焦距：基于相邻两点线性插值得到的值
     *
     * 逻辑：
     * - 第1个和最后1个点不做剔除（边界点，没有双侧邻居）
     * - 中间点：用前后两邻居线性插值计算预期焦距，残差 > 3σ 则剔除
     * - 最多 3 轮迭代
     * - 保留至少 60% 的样本
     */
    private static FocalCalibrationResult filterOutliers(List<FocalCalibrationPoint> sorted, OpticalDevice device) {
        if (sorted.size() < 4) {
            // 样本太少，不做剔除
            log.info("样本数 < 4，跳过野值剔除");
            return new FocalCalibrationResult(sorted, 0, device);
        }

        List<FocalCalibrationPoint> current = new ArrayList<>(sorted);
        int removedTotal = 0;

        for (int round = 0; round < 3; round++) {
            // 计算残差
            List<Double> residuals = new ArrayList<>();
            List<FocalCalibrationPoint> candidates = new ArrayList<>();

            for (int i = 1; i < current.size() - 1; i++) {
                FocalCalibrationPoint prev = current.get(i - 1);
                FocalCalibrationPoint curr = current.get(i);
                FocalCalibrationPoint next = current.get(i + 1);

                // 线性插值预期焦距
                double t = (curr.distanceM() - prev.distanceM()) / (next.distanceM() - prev.distanceM());
                double expectedFocal = prev.focalLengthMm() + t * (next.focalLengthMm() - prev.focalLengthMm());
                double residual = Math.abs(curr.focalLengthMm() - expectedFocal);

                residuals.add(residual);
                candidates.add(curr);
            }

            if (residuals.isEmpty()) break;

            // 计算残差的标准差
            double meanResidual = residuals.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
            double stdResidual = Math.sqrt(
                    residuals.stream().mapToDouble(r -> Math.pow(r - meanResidual, 2)).sum() / residuals.size());

            // 标准差为 0 说明所有点完美线性，无需剔除
            if (stdResidual < 1e-10) break;

            // 3σ 剔除
            List<FocalCalibrationPoint> newCurrent = new ArrayList<>();
            newCurrent.add(current.get(0)); // 保留第1个点

            int removed = 0;
            for (int i = 0; i < candidates.size(); i++) {
                if (residuals.get(i) > 3 * stdResidual) {
                    log.warning(String.format("野值剔除(第%d轮): 距离=%.1fm, 焦距=%.2fmm, 残差=%.4fmm (> 3σ=%.4fmm)",
                            round + 1, candidates.get(i).distanceM(), candidates.get(i).focalLengthMm(),
                            residuals.get(i), 3 * stdResidual));
                    removed++;
                } else {
                    newCurrent.add(candidates.get(i));
                }
            }

            newCurrent.add(current.get(current.size() - 1)); // 保留最后1个点

            if (removed == 0) break;
            removedTotal += removed;

            // 保留至少 60%
            if (newCurrent.size() < sorted.size() * 0.6) {
                log.warning(String.format("野值剔除超过 40%%，停止迭代: 原始 %d → 当前 %d",
                        sorted.size(), newCurrent.size()));
                break;
            }

            current = newCurrent;
        }

        return new FocalCalibrationResult(current, removedTotal, device);
    }

    // ==================== 保存：独立输出 ====================

    /**
     * 保存为独立的焦距表 JSON 文件
     *
     * 格式示例：
     * <pre>
     * {
     *   "focal_length_table": {
     *     "unit": "mm",
     *     "distance_unit": "m",
     *     "entries": [
     *       { "distance": 500, "focal_length": 200.0 },
     *       ...
     *     ]
     *   },
     *   "calibration_info": { ... }
     * }
     * </pre>
     */
    public static void saveStandalone(FocalCalibrationResult result, String outputPath) throws IOException {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");

        // 焦距表
        sb.append("  \"focal_length_table\": {\n");
        sb.append("    \"unit\": \"mm\",\n");
        sb.append("    \"distance_unit\": \"m\",\n");
        sb.append("    \"description\": \"距离(米)→焦距(mm)查表，按距离升序，线性插值\",\n");
        sb.append("    \"entries\": [\n");

        for (int i = 0; i < result.validPoints().size(); i++) {
            FocalCalibrationPoint p = result.validPoints().get(i);
            sb.append(String.format("      { \"distance\": %.1f, \"focal_length\": %.1f }",
                    p.distanceM(), p.focalLengthMm()));
            if (i < result.validPoints().size() - 1) sb.append(",");
            sb.append("\n");
        }

        sb.append("    ]\n");
        sb.append("  },\n");

        // 标定信息
        sb.append("  \"calibration_info\": {\n");
        sb.append("    \"date\": \"").append(LocalDate.now()).append("\",\n");
        sb.append("    \"valid_point_count\": ").append(result.validPoints().size()).append(",\n");
        sb.append("    \"removed_outlier_count\": ").append(result.removedCount()).append(",\n");

        if (result.device() != null) {
            OpticalDevice dev = result.device();
            sb.append("    \"optical_device\": {\n");
            boolean hasField = false;
            if (dev.model() != null) {
                sb.append("      \"model\": \"").append(dev.model()).append("\"");
                hasField = true;
            }
            if (dev.focalLengthMinMm() != null) {
                if (hasField) sb.append(",");
                sb.append("\n      \"focal_length_min_mm\": ").append(dev.focalLengthMinMm());
                hasField = true;
            }
            if (dev.focalLengthMaxMm() != null) {
                if (hasField) sb.append(",");
                sb.append("\n      \"focal_length_max_mm\": ").append(dev.focalLengthMaxMm());
                hasField = true;
            }
            if (hasField) sb.append("\n");
            sb.append("    }\n");
        } else {
            sb.append("    \"optical_device\": null\n");
        }

        sb.append("  }\n");
        sb.append("}\n");

        Files.writeString(Path.of(outputPath), sb.toString());
        log.info("焦距表已保存: " + outputPath);
    }

    // ==================== 保存：合并到 calibration_v2.json ====================

    /**
     * 将焦距表合并到现有 calibration_v2.json
     *
     * 行为：
     * - 读取现有 calibration_v2.json
     * - 在 parameters 下添加/替换 focal_length_table 节点
     * - 保留所有其他字段不变
     * - 原子性：先写临时文件，再重命名
     */
    public static void mergeToCalibrationV2(FocalCalibrationResult result, String calibrationV2Path) throws IOException {
        Path targetPath = Path.of(calibrationV2Path);

        if (!Files.exists(targetPath)) {
            throw new IOException("calibration_v2.json 不存在: " + calibrationV2Path);
        }

        String json = Files.readString(targetPath);
        Map<String, Object> raw = SimpleJsonParser.parse(json);

        @SuppressWarnings("unchecked")
        Map<String, Object> params = (Map<String, Object>) raw.get("parameters");
        if (params == null) {
            throw new IOException("calibration_v2.json 缺少 parameters 节点");
        }

        // 构建 focal_length_table 节点
        Map<String, Object> table = new LinkedHashMap<>();
        table.put("unit", "mm");
        table.put("distance_unit", "m");
        table.put("description", "距离(米)→焦距(mm)查表，按距离升序，线性插值");

        List<Map<String, Object>> entries = new ArrayList<>();
        for (FocalCalibrationPoint p : result.validPoints()) {
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("distance", p.distanceM());
            entry.put("focal_length", p.focalLengthMm());
            entries.add(entry);
        }
        table.put("entries", entries);

        // 添加/替换到 parameters
        if (params.containsKey("focal_length_table")) {
            log.warning("calibration_v2.json 已存在 focal_length_table，将被覆盖");
        }
        params.put("focal_length_table", table);

        // 重新序列化（保持简易 JSON 格式，与 CalibrateV2 风格一致）
        String updatedJson = rebuildJson(raw);
        Files.writeString(targetPath, updatedJson);
        log.info("焦距表已合并到: " + calibrationV2Path);
    }

    // ==================== JSON 序列化 ====================

    /**
     * 将解析后的 Map 重新序列化为 JSON
     *
     * 保持与 CalibrateV2.saveCalibrationV2() 相同的手工拼接风格，
     * 确保输出格式一致。
     */
    private static String rebuildJson(Map<String, Object> root) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        buildJsonObject(sb, root, 1);
        sb.append("}\n");
        return sb.toString();
    }

    /**
     * 递归构建 JSON 对象
     */
    @SuppressWarnings("unchecked")
    private static void buildJsonObject(StringBuilder sb, Map<String, Object> map, int indent) {
        String pad = "  ".repeat(indent);
        String padInner = "  ".repeat(indent + 1);

        int i = 0;
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            sb.append(padInner).append("\"").append(entry.getKey()).append("\": ");

            Object value = entry.getValue();
            if (value == null) {
                sb.append("null");
            } else if (value instanceof Map) {
                sb.append("{\n");
                buildJsonObject(sb, (Map<String, Object>) value, indent + 2);
                sb.append(padInner).append("}");
            } else if (value instanceof List) {
                sb.append(buildJsonArray((List<Object>) value, indent + 1));
            } else if (value instanceof String) {
                sb.append("\"").append(escapeJsonString((String) value)).append("\"");
            } else if (value instanceof Boolean) {
                sb.append(value);
            } else if (value instanceof Number) {
                sb.append(formatNumber((Number) value));
            }

            if (++i < map.size()) sb.append(",");
            sb.append("\n");
        }
    }

    /**
     * 构建 JSON 数组
     */
    @SuppressWarnings("unchecked")
    private static String buildJsonArray(List<Object> list, int indent) {
        StringBuilder sb = new StringBuilder();
        String pad = "  ".repeat(indent);
        String padInner = "  ".repeat(indent + 1);

        sb.append("[\n");
        for (int i = 0; i < list.size(); i++) {
            Object item = list.get(i);
            sb.append(padInner);

            if (item instanceof Map) {
                sb.append("{\n");
                buildJsonObject(sb, (Map<String, Object>) item, indent + 2);
                sb.append(padInner).append("}");
            } else if (item instanceof String) {
                sb.append("\"").append(escapeJsonString((String) item)).append("\"");
            } else if (item instanceof Number) {
                sb.append(formatNumber((Number) item));
            } else if (item instanceof Boolean) {
                sb.append(item);
            } else if (item == null) {
                sb.append("null");
            }

            if (i < list.size() - 1) sb.append(",");
            sb.append("\n");
        }
        sb.append(pad).append("]");

        return sb.toString();
    }

    /**
     * 格式化数字：整数不带小数点，浮点数保留必要精度
     */
    private static String formatNumber(Number n) {
        if (n instanceof Long || n instanceof Integer) {
            return n.toString();
        }
        double d = n.doubleValue();
        if (d == Math.floor(d) && !Double.isInfinite(d)) {
            return String.valueOf((long) d);
        }
        // 保留6位有效小数（与 CalibrateV2 的 round6 风格一致）
        return String.valueOf(Math.round(d * 1_000_000.0) / 1_000_000.0);
    }

    /**
     * 转义 JSON 字符串中的特殊字符
     */
    private static String escapeJsonString(String s) {
        return s.replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r")
                .replace("\t", "\\t");
    }

    // ==================== 工具方法 ====================

    private static double toDouble(Object obj) {
        if (obj instanceof Number n) return n.doubleValue();
        return Double.parseDouble(obj.toString());
    }
}
