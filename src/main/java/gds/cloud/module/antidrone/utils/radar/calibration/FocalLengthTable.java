package gds.cloud.module.antidrone.utils.radar.calibration;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * 距离→焦距查表（线性插值）
 *
 * 物理依据：焦距与斜距呈线性关系 f = D × (sensor_size / desired_image_size)，
 * 因此线性插值在目标尺寸固定时是精确的。
 *
 * 距离基准：3D斜距（ECEF欧氏距离），即 opticalToTargetRange，
 * 与雷达补偿分段使用的 haversine 水平距离不同。
 */
public class FocalLengthTable {

    private final List<Entry> entries;

    public record Entry(double distance, double focalLength) implements Comparable<Entry> {
        @Override
        public int compareTo(Entry o) {
            return Double.compare(this.distance, o.distance);
        }
    }

    private FocalLengthTable(List<Entry> entries) {
        this.entries = Collections.unmodifiableList(new ArrayList<>(entries));
    }

    /**
     * 从条目列表构造（条目会按距离排序并校验严格递增）
     *
     * @throws IllegalArgumentException 如果距离不严格递增
     */
    public static FocalLengthTable of(List<Entry> entries) {
        Objects.requireNonNull(entries, "entries must not be null");
        List<Entry> sorted = new ArrayList<>(entries);
        Collections.sort(sorted);
        for (int i = 1; i < sorted.size(); i++) {
            if (Double.compare(sorted.get(i).distance(), sorted.get(i - 1).distance()) <= 0) {
                throw new IllegalArgumentException(
                    "距离必须严格递增: entries[" + (i - 1) + "].distance=" + sorted.get(i - 1).distance()
                    + ", entries[" + i + "].distance=" + sorted.get(i).distance());
            }
        }
        return new FocalLengthTable(sorted);
    }

    /** 空表（lookup 始终返回 0.0） */
    public static FocalLengthTable empty() {
        return new FocalLengthTable(List.of());
    }

    /**
     * 根据斜距查表求焦距
     *
     * - 空表 → 0.0
     * - 距离 ≤ 表最小值 → 最小距离对应的焦距
     * - 距离 ≥ 表最大值 → 最大距离对应的焦距
     * - 中间值 → 线性插值
     *
     * @param distanceMeters 光电到目标的3D斜距（米）
     * @return 焦距（mm），空表返回 0.0
     */
    public double lookup(double distanceMeters) {
        if (entries.isEmpty()) {
            return 0.0;
        }

        if (distanceMeters <= entries.get(0).distance()) {
            return entries.get(0).focalLength();
        }

        if (distanceMeters >= entries.get(entries.size() - 1).distance()) {
            return entries.get(entries.size() - 1).focalLength();
        }

        int idx = Collections.binarySearch(entries, new Entry(distanceMeters, 0));
        if (idx >= 0) {
            return entries.get(idx).focalLength();
        }

        // binarySearch 返回 -(insertionPoint) - 1
        int insertionPoint = -idx - 1;
        Entry lower = entries.get(insertionPoint - 1);
        Entry upper = entries.get(insertionPoint);

        double ratio = (distanceMeters - lower.distance()) / (upper.distance() - lower.distance());
        return lower.focalLength() + ratio * (upper.focalLength() - lower.focalLength());
    }

    /** 是否为空表 */
    public boolean isEmpty() {
        return entries.isEmpty();
    }

    /** 获取条目数 */
    public int size() {
        return entries.size();
    }

    /** 获取条目列表（只读） */
    public List<Entry> entries() {
        return entries;
    }
}
