package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.CoordinateUtils;

/**
 * 深入调试 - 分析往返误差来源
 */
public class DebugTest2 {

    public static void main(String[] args) {
        System.out.println("=".repeat(60));
        System.out.println("       往返误差来源分析");
        System.out.println("=".repeat(60));

        StationBLH optical = new StationBLH(39.9043, 116.4075, 51.0);

        // 测试不同距离和高度组合
        double[] distances = {1.0, 2.0, 3.0, 5.0};
        double[] heights = {50.0, 100.0, 250.0, 500.0};
        double az = 45.0;

        System.out.println("\n[1] 不同距离/高度组合的误差分析");
        System.out.println("    (原始方法: 使用水平距离近似)");
        System.out.println("-".repeat(55));
        System.out.printf("    %-10s  %-10s  %-12s  %-12s%n", "距离(km)", "高度(m)", "Az误差(°)", "El误差(°)");
        System.out.println("-".repeat(55));

        for (double dist : distances) {
            for (double h : heights) {
                double[] errors = testRoundTrip(optical, dist, az, h);
                System.out.printf("    %-10.1f  %-10.1f  %-12.6f  %-12.6f%n", dist, h, errors[0], errors[1]);
            }
            System.out.println();
        }

        System.out.println("=".repeat(60));
    }

    private static double[] testRoundTrip(StationBLH optical, double distKm, double az, double height) {
        // 原始方法：使用水平距离
        double azRad = Math.toRadians(az);
        double distRad = distKm / 6371.0;  // 近似

        double lat1 = Math.toRadians(optical.B());
        double lon1 = Math.toRadians(optical.L());

        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(distRad) +
                                Math.cos(lat1) * Math.sin(distRad) * Math.cos(azRad));
        double lon2 = lon1 + Math.atan2(Math.sin(azRad) * Math.sin(distRad) * Math.cos(lat1),
                                        Math.cos(distRad) - Math.sin(lat1) * Math.sin(lat2));

        StationBLH target = new StationBLH(Math.toDegrees(lat2), Math.toDegrees(lon2), height);

        double[] recoveredAzEl = CoordinateUtils.blhToAzEl(
            target.B(), target.L(), target.H(),
            optical.B(), optical.L(), optical.H()
        );

        double azErr = Math.abs(recoveredAzEl[0] - az);
        if (azErr > 180) azErr = 360 - azErr;
        double elErr = Math.abs(recoveredAzEl[1]);

        return new double[]{azErr, elErr};
    }
}
