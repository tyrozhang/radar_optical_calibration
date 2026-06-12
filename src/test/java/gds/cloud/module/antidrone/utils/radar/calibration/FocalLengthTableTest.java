package gds.cloud.module.antidrone.utils.radar.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

/**
 * FocalLengthTable 单元测试
 */
public class FocalLengthTableTest {

    private static final Logger log = Logger.getLogger(FocalLengthTableTest.class.getName());
    private static int passed = 0;
    private static int failed = 0;

    public static void main(String[] args) {
        testExactMatch();
        testLinearInterpolation();
        testLowerClamp();
        testUpperClamp();
        testEmptyTable();
        testSingleEntry();
        testUnsortedInput();
        testDescendingDistanceRejected();

        log.info(String.format("=== 测试结果: %d 通过, %d 失败 ===", passed, failed));
        if (failed > 0) System.exit(1);
    }

    private static void testExactMatch() {
        log.info("【测试1】精确匹配");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(500, 200.0),
            new FocalLengthTable.Entry(1000, 135.0),
            new FocalLengthTable.Entry(2000, 80.0)
        ));

        assertEqual("精确匹配 500m", table.lookup(500), 200.0);
        assertEqual("精确匹配 1000m", table.lookup(1000), 135.0);
        assertEqual("精确匹配 2000m", table.lookup(2000), 80.0);
    }

    private static void testLinearInterpolation() {
        log.info("【测试2】线性插值");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(500, 200.0),
            new FocalLengthTable.Entry(1000, 135.0)
        ));

        // 750m: midpoint → (200 + 135) / 2 = 167.5
        assertEqual("插值 750m", table.lookup(750), 167.5);
        // 600m: ratio = (600-500)/(1000-500) = 0.2 → 200 + 0.2*(135-200) = 200 - 13 = 187
        assertEqual("插值 600m", table.lookup(600), 187.0);
    }

    private static void testLowerClamp() {
        log.info("【测试3】下界 clamp");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(500, 200.0),
            new FocalLengthTable.Entry(1000, 135.0)
        ));

        assertEqual("距离<最小值", table.lookup(100), 200.0);
        assertEqual("距离=0", table.lookup(0), 200.0);
        assertEqual("距离<0", table.lookup(-50), 200.0);
    }

    private static void testUpperClamp() {
        log.info("【测试4】上界 clamp");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(500, 200.0),
            new FocalLengthTable.Entry(1000, 135.0)
        ));

        assertEqual("距离>最大值", table.lookup(5000), 135.0);
        assertEqual("距离=10000", table.lookup(10000), 135.0);
    }

    private static void testEmptyTable() {
        log.info("【测试5】空表");
        var table = FocalLengthTable.empty();

        assertEqual("空表查表", table.lookup(1000), 0.0);
        assertTrue("空表 isEmpty", table.isEmpty());
        assertEqual("空表 size", table.size(), 0);
    }

    private static void testSingleEntry() {
        log.info("【测试6】单条目表");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(1000, 50.0)
        ));

        assertEqual("单条目 低于", table.lookup(500), 50.0);
        assertEqual("单条目 精确", table.lookup(1000), 50.0);
        assertEqual("单条目 高于", table.lookup(2000), 50.0);
    }

    private static void testUnsortedInput() {
        log.info("【测试7】乱序输入自动排序");
        var table = FocalLengthTable.of(List.of(
            new FocalLengthTable.Entry(2000, 80.0),
            new FocalLengthTable.Entry(500, 200.0),
            new FocalLengthTable.Entry(1000, 135.0)
        ));

        assertEqual("排序后插值 750m", table.lookup(750), 167.5);
    }

    private static void testDescendingDistanceRejected() {
        log.info("【测试8】重复距离被拒绝");
        boolean caught = false;
        try {
            FocalLengthTable.of(List.of(
                new FocalLengthTable.Entry(500, 200.0),
                new FocalLengthTable.Entry(500, 150.0)
            ));
        } catch (IllegalArgumentException e) {
            caught = true;
            log.info("正确抛出异常: " + e.getMessage());
        }
        assertTrue("重复距离抛出异常", caught);
    }

    // ==================== 断言工具 ====================

    private static void assertEqual(String name, double actual, double expected) {
        if (Math.abs(actual - expected) < 1e-9) {
            passed++;
            log.info("  ✓ " + name + ": " + actual);
        } else {
            failed++;
            log.warning("  ✗ " + name + ": expected=" + expected + ", actual=" + actual);
        }
    }

    private static void assertEqual(String name, int actual, int expected) {
        if (actual == expected) {
            passed++;
            log.info("  ✓ " + name + ": " + actual);
        } else {
            failed++;
            log.warning("  ✗ " + name + ": expected=" + expected + ", actual=" + actual);
        }
    }

    private static void assertTrue(String name, boolean condition) {
        if (condition) {
            passed++;
            log.info("  ✓ " + name);
        } else {
            failed++;
            log.warning("  ✗ " + name);
        }
    }
}
