package gds.cloud.module.antidrone.utils.radar.calibration;

/**
 * TrackerInvoker 测试
 */
public class TrackerInvokerTest {

    public static void main(String[] args) {
        System.out.println("========== TrackerInvoker 测试 ==========\n");

        // 1. 全局默认实例
        TrackerInvoker invoker = TrackerInvoker.getDefault();
        invoker.clear(); // 清空状态

        // 2. 注册 tracker-001
        System.out.println("[测试] 注册 tracker-001");
        invoker.register("tracker-001", "calibration_v2.json");

        // 3. 幂等测试：重复注册不重建
        System.out.println("\n[测试] 重复注册 tracker-001（应复用）");
        invoker.register("tracker-001", "calibration_v2.json");

        // 4. 强制重建
        System.out.println("\n[测试] 强制重建 tracker-001");
        invoker.registerForce("tracker-001", "calibration_v2.json");

        // 5. 列出所有 key
        System.out.println("\n[测试] 列出已注册: " + invoker.listKeys());

        // 6. 计算指令
        System.out.println("\n[测试] 计算指令 B=39.905 L=116.408 H=100");
        var cmd = invoker.compute("tracker-001", 39.905, 116.408, 100.0, 5000.0, 2.5);
        System.out.printf("  → Az=%.6f, El=%.6f%n", cmd.azCmd(), cmd.elCmd());
        System.out.printf("  → 光电距目标: %.2fm | 宽: %.2fm | 高: %.2fm%n",
            cmd.opticalToTargetRange(), cmd.targetWidth(), cmd.targetHeight());

        // 7. 未注册 key 异常
        System.out.println("\n[测试] 未注册 key 异常");
        try {
            invoker.compute("not-exists", 39.905, 116.408, 100.0, null, null);
            System.out.println("  ❌ 预期异常但未抛出");
        } catch (IllegalArgumentException e) {
            System.out.println("  ✅ 正确抛出: " + e.getMessage());
        }

        // 8. get() 返回 Optional
        System.out.println("\n[测试] Optional get");
        invoker.get("tracker-001").ifPresent(t ->
            System.out.println("  ✅ tracker-001 存在"));
        invoker.get("not-exists").ifPresentOrElse(
            t -> System.out.println("  ❌ 不应存在"),
            () -> System.out.println("  ✅ not-exists 不存在（正确）"));

        System.out.println("\n========== 测试完成 ==========");
    }
}
