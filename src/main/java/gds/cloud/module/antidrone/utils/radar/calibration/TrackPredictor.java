package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;

/**
 * 基于真实速度的航迹预测器
 *
 * 【解决问题】
 * 雷达2秒给一个信号，直接跳变导致光电画面跳跃
 *
 * 【核心算法】
 * 1. 利用雷达提供的真实速度数据（velEast/North/Up）
 * 2. 基于UTC时间戳进行线性外推
 * 3. 将预测后的BLH位置转换为光电指向角(AzEl)
 *
 * 【坐标系约定】
 * - velEast/North/Up：站心坐标系，单位m/s
 *   + East: 向东为正
 *   + North: 向北为正
 *   + Up: 向上为正
 * - 速度数据由雷达航迹接口提供
 *
 * 【使用方式】
 * - 雷达每2秒更新一次航迹点（包含位置+速度）
 * - 光电控制周期50-100ms查询预测角度（弥补2秒间隔）
 */
public class TrackPredictor {

    // ==================== 可调参数 ====================

    /** 最大预测时长（秒），防止外推过远导致误差累积 */
    public final double maxPredictionSeconds = 2.0;

    /** 速度变化阈值（m/s²），超过此值认为是机动目标 */
    public final double maneuverAccelThreshold = 5.0;

    // ==================== 内部状态 ====================

    private long lastTimestamp;                          // 上次雷达更新时间（毫秒）
    private double lastTargetB, lastTargetL, lastTargetH; // 上次目标位置（度/米）
    private double velEast, velNorth, velUp;              // 当前速度（米/秒）
    private double prevVelEast, prevVelNorth, prevVelUp;  // 上次速度（用于机动检测）
    private StationBLH opticalBlh;                        // 光电位置
    private boolean initialized = false;
    private boolean maneuverDetected = false;            // 是否检测到机动

    // ==================== 构造函数 ====================

    /**
     * 构造函数
     *
     * @param opticalBlh 光电跟踪站位置（大地坐标BLH）
     */
    public TrackPredictor(StationBLH opticalBlh) {
        this.opticalBlh = opticalBlh;
    }

    // ==================== 核心方法：航迹更新 ====================

    /**
     * 航迹更新（由雷达数据触发，约2秒一次）
     *
     * 【注意】速度数据必须来自雷达航迹接口（如AIS/ADS-B/军用雷达）
     * 速度坐标系：站心坐标系 East/North/Up
     *
     * @param timestamp   UTC时间戳（毫秒），使用 System.currentTimeMillis()
     * @param targetB     目标纬度（度）
     * @param targetL     目标经度（度）
     * @param targetH     目标高度（米）
     * @param velEast     东向速度（米/秒），+表示向东
     * @param velNorth    北向速度（米/秒），+表示向北
     * @param velUp       天向速度（米/秒），+表示向上
     */
    public void update(
            long timestamp,
            double targetB, double targetL, double targetH,
            double velEast, double velNorth, double velUp) {

        // ========== 机动检测 ==========
        // 如果有历史速度，检查加速度是否超限
        if (initialized) {
            double dt = (timestamp - lastTimestamp) / 1000.0; // 秒
            if (dt > 0.01) {
                double accelEast  = (velEast  - this.velEast)  / dt;
                double accelNorth = (velNorth - this.velNorth) / dt;
                double accelUp    = (velUp    - this.velUp)    / dt;
                double totalAccel = Math.sqrt(accelEast*accelEast + accelNorth*accelNorth + accelUp*accelUp);

                // 加速度超过阈值，认为是机动
                maneuverDetected = totalAccel > maneuverAccelThreshold;
            }
        }

        // 保存上次速度用于下次机动检测
        this.prevVelEast  = this.velEast;
        this.prevVelNorth = this.velNorth;
        this.prevVelUp    = this.velUp;

        // 更新状态
        this.lastTimestamp = timestamp;
        this.lastTargetB   = targetB;
        this.lastTargetL   = targetL;
        this.lastTargetH   = targetH;
        this.velEast  = velEast;
        this.velNorth = velNorth;
        this.velUp    = velUp;
        this.initialized = true;
    }

    /**
     * 简化版航迹更新（无速度数据时使用，用位置差分估算速度）
     *
     * 【适用场景】
     * 雷达只提供位置数据，不提供速度数据
     * 通过连续两次位置计算速度
     *
     * 【精度说明】
     * - 在小范围(<1km)内，BLH差分≈线性位移，精度可接受
     * - 比直接用ECEF坐标差分更准确（避免ECEF轴向问题）
     *
     * @param timestamp UTC时间戳（毫秒）
     * @param targetB  目标纬度（度）
     * @param targetL  目标经度（度）
     * @param targetH  目标高度（米）
     */
    public void update(long timestamp, double targetB, double targetL, double targetH) {
        if (initialized) {
            double dt = (timestamp - lastTimestamp) / 1000.0; // 秒
            if (dt > 0.01) {
                // ========== 用BLH差分估算站心速度 ==========
                //
                // 关键：不能用ECEF差分估算东/北/天速度！
                // 原因：ECEF坐标系的地心地固轴(X指向0°经线)与站心轴(东向)不重合
                //
                // 正确方法：用BLH差分
                // - 纬度1度 ≈ 111132米（与经度无关）
                // - 经度1度 = 111132 × cos(B) 米（随纬度变化）
                //
                double B_rad = Math.toRadians(lastTargetB);
                double latDegToMeters = 111132.0;                            // 纬度→米（固定值）
                double lonDegToMeters = 111132.0 * Math.cos(B_rad);          // 经度→米（随纬度变化）

                double newVelNorth = (targetB - lastTargetB) * latDegToMeters / dt;
                double newVelEast  = (targetL - lastTargetL) * lonDegToMeters / dt;
                double newVelUp    = (targetH - lastTargetH) / dt;

                // 机动检测
                double accelNorth = (newVelNorth - this.velNorth) / dt;
                double accelEast  = (newVelEast  - this.velEast)  / dt;
                double accelUp    = (newVelUp    - this.velUp)    / dt;
                double totalAccel = Math.sqrt(accelEast*accelEast + accelNorth*accelNorth + accelUp*accelUp);
                maneuverDetected = totalAccel > maneuverAccelThreshold;

                // 保存上次速度
                this.prevVelEast  = this.velEast;
                this.prevVelNorth = this.velNorth;
                this.prevVelUp    = this.velUp;

                // 更新速度
                this.velNorth = newVelNorth;
                this.velEast  = newVelEast;
                this.velUp    = newVelUp;
            }
        }

        this.lastTimestamp = timestamp;
        this.lastTargetB   = targetB;
        this.lastTargetL   = targetL;
        this.lastTargetH   = targetH;
        this.initialized = true;
    }

    // ==================== 核心方法：获取预测角度 ====================

    /**
     * 获取预测角度（任意时刻）
     *
     * 【算法】
     * 1. 计算时间差 dt（秒），不超过 maxPredictionSeconds
     * 2. 线性外推：predBLH = lastBLH + vel × dt
     * 3. 调用 CoordinateUtils.blhToAzEl() 转换为光电指向角
     *
     * 【精度说明】
     * - 假设目标做匀速直线运动
     * - 对于2秒以内的预测，误差通常在可接受范围
     * - 机动目标（转弯/加速）会导致误差增大
     *
     * @param currentTimeMs 当前UTC时间戳（毫秒）
     * @return 预测的光电指向角 [Az, El]，单位：度
     * @throws IllegalStateException 未初始化时抛出
     */
    public double[] getPredictedAngles(long currentTimeMs) {
        if (!initialized) {
            throw new IllegalStateException("未初始化，请先调用update()");
        }

        // ========== 时间处理 ==========
        double dt = (currentTimeMs - lastTimestamp) / 1000.0; // 转为秒
        if (dt < 0) {
            // 时间倒流保护：返回当前时刻（不外推）
            dt = 0;
        }
        dt = Math.min(dt, maxPredictionSeconds); // 限制预测时长

        // ========== 速度换算：m/s → 度/米 ==========
        //
        // 关键：经度方向的距离与纬度有关！
        // 示例：北纬39度处
        //   - 纬度1度 ≈ 111132米（固定）
        //   - 经度1度 ≈ 111132 × cos(39°) ≈ 86300米（比纬度短）
        //
        double B_rad = Math.toRadians(lastTargetB);
        double latDegToMeters = 111132.0;                          // 纬度→米
        double lonDegToMeters = 111132.0 * Math.cos(B_rad);       // 经度→米（随纬度变化）

        // 速度(m/s) × 时间(s) ÷ 距离换算 = 角度增量(度)
        double deltaB = velNorth / latDegToMeters * dt;  // 纬度增量（度）
        double deltaL = velEast  / lonDegToMeters * dt;  // 经度增量（度）
        double deltaH = velUp * dt;                      // 高度增量（米）

        // ========== 线性外推位置 ==========
        double predB = lastTargetB + deltaB;
        double predL = lastTargetL + deltaL;
        double predH = lastTargetH + deltaH;

        // ========== 转换为光电指向角 ==========
        double[] azEl = CoordinateUtils.blhToAzEl(
            predB, predL, predH,
            opticalBlh.B(), opticalBlh.L(), opticalBlh.H()
        );

        return new double[]{azEl[0], azEl[1]}; // [Az, El]
    }

    // ==================== 辅助方法 ====================

    /**
     * 检查预测是否有效
     *
     * @param currentTimeMs 当前UTC时间戳（毫秒）
     * @return true=有效，false=已过期
     */
    public boolean isPredictionValid(long currentTimeMs) {
        if (!initialized) return false;
        double dt = (currentTimeMs - lastTimestamp) / 1000.0;
        return dt <= maxPredictionSeconds;
    }

    /**
     * 检查目标是否有机动
     *
     * 【用途】
     * - 可用于降低预测权重或切换算法
     * - 机动检测后，建议减小预测时长或直接使用雷达数据
     *
     * @return true=检测到机动，false=匀速运动
     */
    public boolean isManeuverDetected() {
        return maneuverDetected;
    }

    /**
     * 获取上次雷达更新时间
     *
     * 【用途】
     * - 判断数据是否新鲜
     * - 与光电控制周期配合使用
     *
     * @return 上次更新的UTC时间戳（毫秒）
     */
    public long getLastUpdateTime() {
        return lastTimestamp;
    }

    /**
     * 重置跟踪器状态
     *
     * 【调用时机】
     * - 目标丢失后
     * - 切换跟踪目标时
     */
    public void reset() {
        initialized = false;
        lastTimestamp = 0;
        maneuverDetected = false;
    }

    // ==================== 测试用例 ====================

    public static void main(String[] args) {
        System.out.println("\n========== 航迹预测器测试 ==========\n");

        // 模拟光电跟踪站
        StationBLH opticalBlh = new StationBLH(39.905, 116.408, 50);

        // 创建预测器
        TrackPredictor predictor = new TrackPredictor(opticalBlh);

        System.out.println("测试场景：目标匀速移动，雷达2秒更新，光电50ms查询\n");

        // 打印表头
        System.out.printf("%-6s | %-12s | %-12s | %-10s | %s%n",
            "时间", "预测Az", "预测El", "距离(m)", "备注");
        System.out.println("-".repeat(65));

        long baseTime = System.currentTimeMillis();

        // 模拟目标匀速移动（东北方向，10m/s）
        double velEast  = 7.07;   // 10 × cos(45°)
        double velNorth = 7.07;   // 10 × sin(45°)
        double velUp    = 0;

        // 初始位置
        double targetB = 39.905, targetL = 116.408, targetH = 200;

        // ========== 第一步：雷达首次更新 ==========
        long radarTime1 = baseTime;
        predictor.update(radarTime1, targetB, targetL, targetH, velEast, velNorth, velUp);

        // ========== 第二步：光电连续查询（每50ms一次） ==========
        for (int i = 0; i <= 40; i++) {
            long queryTime = radarTime1 + i * 50;
            double[] angles = predictor.getPredictedAngles(queryTime);

            // 每400ms打印一次（8帧）
            if (i % 8 == 0) {
                String note = (i == 0) ? "←雷达更新" : "";
                System.out.printf("%-6d | %12.4f | %12.4f | %-10s |%s%n",
                    queryTime - radarTime1,
                    angles[0],
                    angles[1],
                    String.format("%.1f", velNorth * (i * 50.0 / 1000.0)), // 估算距离
                    note);
            }
        }

        // ========== 第三步：模拟目标机动（速度突变） ==========
        System.out.println("\n-------- 机动检测测试 --------");
        System.out.println("场景：目标原本向北，突然向东加速（检测机动）\n");

        // 目标突然改变方向
        double newVelEast  = 20.0;  // 速度突变！
        double newVelNorth = 5.0;
        predictor.update(radarTime1 + 2000, targetB + 0.001, targetL + 0.002, targetH,
                         newVelEast, newVelNorth, 0);

        System.out.printf("机动检测结果: %s%n",
            predictor.isManeuverDetected() ? "检测到机动！" : "未检测到机动");
        System.out.printf("原速度: (%.2f, %.2f) m/s → 新速度: (%.2f, %.2f) m/s%n",
            velEast, velNorth, newVelEast, newVelNorth);

        System.out.println("\n-------- 测试完成 --------");
        System.out.println("+ 雷达2秒间隔提供位置+速度");
        System.out.println("+ 光电50ms查询预测角度");
        System.out.println("+ 机动检测识别速度突变");
    }
}
