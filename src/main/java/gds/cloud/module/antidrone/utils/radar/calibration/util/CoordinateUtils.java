package gds.cloud.module.antidrone.utils.radar.calibration.util;

import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;

/**
 * WGS84 坐标转换工具类
 *
 * 支持：
 * - BLH (大地坐标) ↔ ECEF (地心直角坐标)
 * - BLH → ENU (站心坐标) → AzEl (方位角/俯仰角)
 * - 球面距离计算
 */
public class CoordinateUtils {

    // WGS84 椭球参数
    public static final double A = 6378137.0;           // 长半轴 (m)
    public static final double F = 1.0 / 298.257223563; // 扁率
    public static final double E2 = 2 * F - F * F;      // 第一偏心率平方
    public static final double R = 6371000.0;           // 地球平均半径 (m)

    /**
     * BLH 转 ECEF（地心直角坐标）
     */
    public static double[] blhToEcef(double B, double L, double H) {
        double B_rad = Math.toRadians(B);
        double L_rad = Math.toRadians(L);

        double sinB = Math.sin(B_rad);
        double cosB = Math.cos(B_rad);
        double sinL = Math.sin(L_rad);
        double cosL = Math.cos(L_rad);

        // 子午圈曲率半径
        double N = A / Math.sqrt(1 - E2 * sinB * sinB);

        double X = (N + H) * cosB * cosL;
        double Y = (N + H) * cosB * sinL;
        double Z = (N * (1 - E2) + H) * sinB;

        return new double[]{X, Y, Z};
    }

    /**
     * ECEF 转 BLH（大地坐标）
     *
     * 使用 Bowring 迭代法
     */
    public static StationBLH ecefToBlh(double X, double Y, double Z) {
        double p = Math.sqrt(X * X + Y * Y);
        double theta = Math.atan2(Z * A, p * (1 - E2) * A);

        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);

        double B_rad = Math.atan2(Z + (E2 * (1 - E2) * A / (1 - E2 * sinTheta * sinTheta)) * sinTheta * sinTheta * sinTheta,
                                   p - E2 * A / (1 - E2 * sinTheta * sinTheta) * cosTheta * cosTheta * cosTheta);

        double sinB = Math.sin(B_rad);
        double N = A / Math.sqrt(1 - E2 * sinB * sinB);
        double H = p / Math.cos(B_rad) - N;
        double L_rad = Math.atan2(Y, X);

        return new StationBLH(Math.toDegrees(B_rad), Math.toDegrees(L_rad), H);
    }

    /**
     * ECEF 转 ENU（站心东北天坐标）
     */
    public static double[] ecefToEnu(double Xt, double Yt, double Zt,
                                      double Xs, double Ys, double Zs,
                                      double Bs, double Ls) {
        double B_rad = Math.toRadians(Bs);
        double L_rad = Math.toRadians(Ls);

        double dx = Xt - Xs;
        double dy = Yt - Ys;
        double dz = Zt - Zs;

        double sinB = Math.sin(B_rad);
        double cosB = Math.cos(B_rad);
        double sinL = Math.sin(L_rad);
        double cosL = Math.cos(L_rad);

        double E = -sinL * dx + cosL * dy;
        double N = -sinB * cosL * dx - sinB * sinL * dy + cosB * dz;
        double U = cosB * cosL * dx + cosB * sinL * dy + sinB * dz;

        return new double[]{E, N, U};
    }

    /**
     * ENU 转 AzEl（方位角/俯仰角）
     *
     * 俯仰角约定：向上（仰视）为负，向下（俯视）为正
     * 例：目标在正上方 → El = -90°，目标在地平线 → El = 0°
     *
     * @return [Az, El]（度）Az: 0-360°顺时针从北起算, El: 向上为负/向下为正
     */
    public static double[] enuToAzEl(double E, double N, double U) {
        // 方位角（北向顺时针）
        double Az_rad = Math.atan2(E, N);
        double Az = Math.toDegrees(Az_rad);
        if (Az < 0) {
            Az += 360.0;
        }

        // 俯仰角：向上为负，向下为正（对 ENU 中 U>0 表示仰视，取反得到负值）
        double slantRange = Math.sqrt(E * E + N * N);
        double El;
        if (slantRange < 1e-6) {
            El = U > 0 ? -90.0 : 90.0;
        } else {
            double El_rad = Math.atan(U / slantRange);
            El = -Math.toDegrees(El_rad);  // 取反：U>0(仰视)→负，U<0(俯视)→正
        }

        return new double[]{Az, El};
    }

    /**
     * BLH 转 AzEl（直接计算站点视角的方位角/俯仰角）
     */
    public static double[] blhToAzEl(double Bt, double Lt, double Ht,
                                      double Bs, double Ls, double Hs) {
        double[] targetEcef = blhToEcef(Bt, Lt, Ht);
        double[] stationEcef = blhToEcef(Bs, Ls, Hs);

        double[] enu = ecefToEnu(
            targetEcef[0], targetEcef[1], targetEcef[2],
            stationEcef[0], stationEcef[1], stationEcef[2],
            Bs, Ls
        );

        return enuToAzEl(enu[0], enu[1], enu[2]);
    }

    /**
     * Haversine 球面距离计算（水平距离）
     *
     * @return 距离（km）
     */
    public static double haversineDistance(double B1, double L1, double B2, double L2) {
        double B1_rad = Math.toRadians(B1);
        double B2_rad = Math.toRadians(B2);
        double dB = Math.toRadians(B2 - B1);
        double dL = Math.toRadians(L2 - L1);

        double a = Math.sin(dB / 2) * Math.sin(dB / 2) +
                   Math.cos(B1_rad) * Math.cos(B2_rad) *
                   Math.sin(dL / 2) * Math.sin(dL / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return (R / 1000.0) * c; // 返回 km
    }

    /**
     * 计算三维距离
     */
    public static double distance3D(StationBLH p1, StationBLH p2) {
        double[] ecef1 = blhToEcef(p1.B(), p1.L(), p1.H());
        double[] ecef2 = blhToEcef(p2.B(), p2.L(), p2.H());

        double dx = ecef2[0] - ecef1[0];
        double dy = ecef2[1] - ecef1[1];
        double dz = ecef2[2] - ecef1[2];

        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
