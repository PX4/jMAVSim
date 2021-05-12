package me.drton.jmavsim;

/**
 * Updater for the visualizer's simulation state's report.
 */
public class ReportUpdater extends WorldObject {
    private static final long UPDATE_INTERVAL_MS = 200;

    private static final StringBuilder builder = new StringBuilder();
    private static long updateIntervalMs;
    private static long nextUpdateT;
    private final Visualizer3D visualizer;


    public ReportUpdater(World world, Visualizer3D visualizer) {
        super(world);
        this.visualizer = visualizer;
        setUpdateInterval(UPDATE_INTERVAL_MS);
    }

    public static long getUpdateInterval() {
        return ReportUpdater.updateIntervalMs;
    }

    public static void setUpdateInterval(long updateInterval) {
        ReportUpdater.updateIntervalMs = updateInterval;
        ReportUpdater.nextUpdateT = System.currentTimeMillis() + updateInterval;
    }

    public static void resetUpdateInterval() {
        setUpdateInterval(UPDATE_INTERVAL_MS);
    }

    @Override
    public void update(long t, boolean paused) {
        if (t < nextUpdateT) {
            return;
        }

        nextUpdateT = t + updateIntervalMs;

        if (!visualizer.showReportText()) {
            return;
        }

        builder.setLength(0);

        for (WorldObject object : getWorld().getObjects()) {
            if (object instanceof ReportingObject) {
                ((ReportingObject) object).report(builder);
            }
        }

        visualizer.setReportText(builder.toString());
    }
}
