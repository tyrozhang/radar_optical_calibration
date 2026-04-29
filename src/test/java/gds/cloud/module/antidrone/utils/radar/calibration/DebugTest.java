package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

/**
 * 调试程序 - 逐步验证坐标计算
 */
public class DebugTest {

    public static void main(String[] args) {
        System.out.println("=".repeat(60));
        System.out.println("       雷达光电联动标定系统 - 调试测试");
        System.out.println("=".repeat(60));

        // 测试站点坐标
        StationBLH optical = new StationBLH(39.9043, 116.4075, 51.0);
        StationBLH radar = new StationBLH(39.9050, 116.4100, 50.0);

        System.out.println("\n[1] 测试站点数据");
        System.out.println("    光电站: " + optical);
        System.out.println("    雷达站: " + radar);

        // 测试坐标转换
        System.out.println("\n[2] 坐标转换验证");
        testBlhToEcef(optical);
        testBlhToAzEl(optical, radar);
        testHaversine(optical, radar);

        // 测试模拟器单点计算
        System.out.println("\n[3] 模拟器单点计算测试");
        testSimulatorPoint(optical, 2.0, 45.0, 100.0);

        // 测试往返一致性
        System.out.println("\n[4] 往返一致性测试");
        testRoundTrip(optical);

        System.out.println("\n" + "=".repeat(60));
        System.out.println("       调试测试完成");
        System.out.println("=".repeat(60));
    }

    private static void testBlhToEcef(StationBLH blh) {
        System.out.println("\n    BLH → ECEF 转换测试:");
        double[] ecef = CoordinateUtils.blhToEcef(blh.B(), blh.L(), blh.H());
        System.out.printf("    输入: B=%.6f°, L=%.6f°, H=%.1fm%n", blh.B(), blh.L(), blh.H());
        System.out.printf("    输出: X=%.3f, Y=%.3f, Z=%.3f%n", ecef[0], ecef[1], ecef[2]);
    }

    private static void testBlhToAzEl(StationBLH station, StationBLH target) {
        System.out.println("\n    角度计算测试 (站→目标):");
        double[] azEl = CoordinateUtils.blhToAzEl(
            target.B(), target.L(), target.H(),
            station.B(), station.L(), station.H()
        );
        System.out.printf("    站点: B=%.6f°, L=%.6f°%n", station.B(), station.L());
        System.out.printf("    目标: B=%.6f°, L=%.6f°%n", target.B(), target.L());
        System.out.printf("    方位角: %.4f°, 俯仰角: %.4f°%n", azEl[0], azEl[1]);
    }

    private static void testHaversine(StationBLH p1, StationBLH p2) {
        System.out.println("\n    Haversine 距离计算测试:");
        double dist = CoordinateUtils.haversineDistance(
            p1.B(), p1.L(), p2.B(), p2.L()
        );
        System.out.printf("    两点水平距离: %.4f km%n", dist);

        double dist3D = CoordinateUtils.distance3D(p1, p2);
        System.out.printf("    两点3D距离: %.4f km%n", dist3D / 1000.0);
    }

    private static void testSimulatorPoint(StationBLH optical, double distKm, double azimuth, double height) {
        System.out.println("\n    模拟器单点计算:");
        System.out.printf("    设定: 距离=%.1fkm, 方位角=%.1f°, 高度=%.1fm%n", distKm, azimuth, height);

        // 1. 球面三角计算目标位置
        double distRad = distKm / 6371.0;  // 注意：这是近似
        double azRad = Math.toRadians(azimuth);
        double lat1 = Math.toRadians(optical.B());
        double lon1 = Math.toRadians(optical.L());

        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(distRad) +
                                Math.cos(lat1) * Math.sin(distRad) * Math.cos(azRad));
        double lon2 = lon1 + Math.atan2(Math.sin(azRad) * Math.sin(distRad) * Math.cos(lat1),
                                        Math.cos(distRad) - Math.sin(lat1) * Math.sin(lat2));

        StationBLH target = new StationBLH(
            Math.toDegrees(lat2),
            Math.toDegrees(lon2),
            height
        );
        System.out.printf("    目标位置: B=%.6f°, L=%.6f°, H=%.1fm%n", target.B(), target.L(), target.H());

        // 2. 从目标反算方位角
        double[] azEl = CoordinateUtils.blhToAzEl(
            target.B(), target.L(), target.H(),
            optical.B(), optical.L(), optical.H()
        );
        System.out.printf("    反算方位角: %.4f° (应为 %.1f°)%n", azEl[0], azimuth);
        System.out.printf("    反算俯仰角: %.4f°%n", azEl[1]);

        // 3. 计算水平距离验证
        double actualDist = CoordinateUtils.haversineDistance(
            optical.B(), optical.L(), target.B(), target.L()
        );
        System.out.printf("    实际水平距离: %.4f km (设定 %.1f km)%n", actualDist, distKm);

        // 误差分析
        double azError = Math.abs(azEl[0] - azimuth);
        if (azError > 180) azError = 360 - azError;
        System.out.printf("    方位角误差: %.4f°%n", azError);
    }

    private static void testRoundTrip(StationBLH optical) {
        System.out.println("\n    往返一致性测试 (AzEl → BLH → AzEl):");

        // 设定一个目标
        double az = 45.0, el = 15.0, distKm = 3.0;
        System.out.printf("    原始设定: Az=%.1f°, El=%.1f°, Dist=%.1fkm%n", az, el, distKm);

        // 1. 从光学站沿给定角度计算目标位置
        double azRad = Math.toRadians(az);
        double elRad = Math.toRadians(el);
        double distM = distKm * 1000;
        double slantRange = distM * Math.cos(elRad);

        double e = slantRange * Math.sin(azRad);
        double n = slantRange * Math.cos(azRad);
        double u = distM * Math.sin(elRad);

        System.out.printf("    ENU坐标: E=%.2f, N=%.2f, U=%.2f (m)%n", e, n, u);

        // 2. ENU → ECEF
        double[] stationEcef = CoordinateUtils.blhToEcef(optical.B(), optical.L(), optical.H());
        double latRad = Math.toRadians(optical.B());
        double lonRad = Math.toRadians(optical.L());

        double sinLat = Math.sin(latRad);
        double cosLat = Math.cos(latRad);
        double sinLon = Math.sin(lonRad);
        double cosLon = Math.cos(lonRad);

        double Xt = stationEcef[0] - sinLon * e - sinLat * cosLon * n + cosLat * cosLon * u;
        double Yt = stationEcef[1] + cosLon * e - sinLat * sinLon * n + cosLat * sinLon * u;
        double Zt = stationEcef[2] + cosLat * n + sinLat * u;

        // 3. ECEF → BLH
        StationBLH target = CoordinateUtils.ecefToBlh(Xt, Yt, Zt);
        System.out.printf("    计算目标: B=%.6f°, L=%.6f°, H=%.1fm%n", target.B(), target.L(), target.H());

        // 4. BLH → AzEl 验证
        double[] recoveredAzEl = CoordinateUtils.blhToAzEl(
            target.B(), target.L(), target.H(),
            optical.B(), optical.L(), optical.H()
        );
        System.out.printf("    恢复方位角: %.4f° (原始 %.1f°)%n", recoveredAzEl[0], az);
        System.out.printf("    恢复俯仰角: %.4f° (原始 %.1f°)%n", recoveredAzEl[1], el);

        // 误差
        double azErr = Math.abs(recoveredAzEl[0] - az);
        if (azErr > 180) azErr = 360 - azErr;
        double elErr = Math.abs(recoveredAzEl[1] - el);
        System.out.printf("    方位角误差: %.6f°%n", azErr);
        System.out.printf("    俯仰角误差: %.6f°%n", elErr);
    }
}
