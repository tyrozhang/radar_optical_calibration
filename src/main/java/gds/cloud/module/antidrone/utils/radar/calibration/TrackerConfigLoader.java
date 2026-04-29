package gds.cloud.module.antidrone.utils.radar.calibration;

import gds.cloud.module.antidrone.utils.radar.calibration.model.RadarSegmentConfig;
import gds.cloud.module.antidrone.utils.radar.calibration.model.StationBLH;
import gds.cloud.module.antidrone.utils.radar.calibration.util.SimpleJsonParser;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * 标定配置加载器
 *
 * 从 calibration_v2.json 解析标定参数，供 RadarOpticTrackerV2 和 SmoothTrackingController 共用。
 */
public class TrackerConfigLoader {

    public static RadarOpticTrackerV2.Config load(String path) {
        try {
            String json = Files.readString(Path.of(path));
            Map<String, Object> raw = SimpleJsonParser.parse(json);

            @SuppressWarnings("unchecked")
            Map<String, Object> params = (Map<String, Object>) raw.get("parameters");

            @SuppressWarnings("unchecked")
            Map<String, Object> opticalMap = (Map<String, Object>) params.get("optical_blh");

            StationBLH opticalBlh = new StationBLH(
                toDouble(opticalMap.get("B")),
                toDouble(opticalMap.get("L")),
                toDouble(opticalMap.get("H"))
            );

            double dAz0 = toDouble(params.get("dAz0"));
            double dEl0 = toDouble(params.get("dEl0"));

            RadarOpticTrackerV2.RadarCompensation radarComp = parseRadarCompensation(params);

            long totalDelayMs = 0L;
            Object delayObj = params.get("radar2opticalTotalDelayMs");
            if (delayObj != null) {
                totalDelayMs = toLong(delayObj);
            }

            System.out.println("[TrackerConfigLoader] 配置加载成功");
            System.out.printf("  光电站: B=%.6f°, L=%.6f°, H=%.1fm%n",
                opticalBlh.B(), opticalBlh.L(), opticalBlh.H());
            System.out.printf("  ΔAz0=%.6f°, ΔEl0=%.6f°%n", dAz0, dEl0);
            System.out.printf("  雷达补偿类型: %s%n",
                radarComp instanceof RadarOpticTrackerV2.FixedCompensation ? "fixed" : "segmented");
            System.out.printf("  总时延: %d ms%n", totalDelayMs);

            return new RadarOpticTrackerV2.Config(opticalBlh, dAz0, dEl0, radarComp, totalDelayMs);
        } catch (IOException e) {
            throw new RadarOpticTrackerV2.TrackingException("加载配置文件失败: " + path, e);
        }
    }

    @SuppressWarnings("unchecked")
    private static RadarOpticTrackerV2.RadarCompensation parseRadarCompensation(Map<String, Object> params) {
        String type = params.get("radar_compensation_type").toString();

        if ("fixed".equals(type)) {
            return new RadarOpticTrackerV2.FixedCompensation(
                toDouble(params.get("dAz_radar")),
                toDouble(params.get("dEl_radar"))
            );
        } else {
            Map<String, Object> compMap = (Map<String, Object>) params.get("radar_compensation");
            List<RadarSegmentConfig> segments = new ArrayList<>();
            for (Object obj : (List<Object>) compMap.get("segments")) {
                Map<String, Object> seg = (Map<String, Object>) obj;
                String range = seg.get("range").toString();
                double min = toDouble(seg.get("min_km"));
                Object maxObj = seg.get("max_km");
                double max = maxObj == null || "null".equals(maxObj.toString())
                    ? Double.MAX_VALUE : toDouble(maxObj);

                segments.add(new RadarSegmentConfig(
                    range, min, max,
                    toDouble(seg.get("dAz")),
                    toDouble(seg.get("dEl"))
                ));
            }
            return new RadarOpticTrackerV2.SegmentedCompensation(segments);
        }
    }

    private static double toDouble(Object obj) {
        if (obj instanceof Number n) return n.doubleValue();
        return Double.parseDouble(obj.toString());
    }

    private static long toLong(Object obj) {
        if (obj instanceof Number n) return n.longValue();
        return Long.parseLong(obj.toString());
    }
}
