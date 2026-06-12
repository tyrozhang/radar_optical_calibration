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
import java.util.logging.Logger;

/**
 * 标定配置加载器
 *
 * 从 calibration_v2.json 解析标定参数，供 RadarOpticTrackerV2 和 SmoothTrackingController 共用。
 */
public class TrackerConfigLoader {

    private static final Logger log = Logger.getLogger(TrackerConfigLoader.class.getName());

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

            // 俯仰角符号约定（可选，默认 up_negative）
            ElevationConvention elevationConvention = ElevationConvention.fromString(
                params.get("elevation_convention") != null
                    ? params.get("elevation_convention").toString()
                    : null
            );

            // 焦距查表（可选，默认空表）
            FocalLengthTable focalLengthTable = parseFocalLengthTable(params);

            log.info(String.format("配置加载成功: 光电站 B=%.6f°, L=%.6f°, H=%.1fm",
                opticalBlh.B(), opticalBlh.L(), opticalBlh.H()));
            log.info(String.format("ΔAz0=%.6f°, ΔEl0=%.6f°", dAz0, dEl0));
            log.info("雷达补偿类型: " +
                (radarComp instanceof RadarOpticTrackerV2.FixedCompensation ? "fixed" : "segmented"));
            log.info("总时延: " + totalDelayMs + " ms");
            log.info("俯仰角约定: " + elevationConvention);
            log.info("焦距表: " + (focalLengthTable.isEmpty() ? "未配置" : focalLengthTable.size() + "个条目"));

            return new RadarOpticTrackerV2.Config(opticalBlh, dAz0, dEl0, radarComp, totalDelayMs, elevationConvention, focalLengthTable);
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

    @SuppressWarnings("unchecked")
    private static FocalLengthTable parseFocalLengthTable(Map<String, Object> params) {
        Object tableObj = params.get("focal_length_table");
        if (tableObj == null) {
            return FocalLengthTable.empty();
        }

        Map<String, Object> tableMap = (Map<String, Object>) tableObj;
        List<Object> entriesRaw = (List<Object>) tableMap.get("entries");
        if (entriesRaw == null || entriesRaw.isEmpty()) {
            return FocalLengthTable.empty();
        }

        List<FocalLengthTable.Entry> entries = new ArrayList<>();
        for (Object obj : entriesRaw) {
            Map<String, Object> entry = (Map<String, Object>) obj;
            entries.add(new FocalLengthTable.Entry(
                toDouble(entry.get("distance")),
                toDouble(entry.get("focal_length"))
            ));
        }

        return FocalLengthTable.of(entries);
    }
}
