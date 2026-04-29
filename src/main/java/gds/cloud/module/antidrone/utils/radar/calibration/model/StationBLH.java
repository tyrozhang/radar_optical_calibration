package gds.cloud.module.antidrone.utils.radar.calibration.model;

/**
 * 站点大地坐标（WGS84）
 *
 * @param B 纬度（度）
 * @param L 经度（度）
 * @param H 高度（米）
 */
public record StationBLH(double B, double L, double H) {

    @Override
    public String toString() {
        return String.format("StationBLH{B=%.6f°, L=%.6f°, H=%.1fm}", B, L, H);
    }
}
