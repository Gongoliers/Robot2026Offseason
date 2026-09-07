package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;

import java.util.Arrays;
import java.util.HashMap;

/**
 * Utility class for publishing poses to NetworkTables.
 */
public class PosePublisher {

    /**
     * Map between a NetworkTables key and the NetworkTables entry where poses are published.
     */
    private static final HashMap<String, StructArrayEntry<Pose3d>> entries = new HashMap<>();

    /**
     * Retrieves the NetworkTables entry associated with a key.
     *
     * @param key The NetworkTables key.
     * @return The NetworkTables entry for that key.
     */
    private static StructArrayEntry<Pose3d> getEntry(String key) {
        // If the entry is already stored in this instance, retrieve the stored entry
        boolean stored = entries.containsKey(key);
        if (stored) {
            return entries.get(key);
        }

        // Otherwise, create the entry using NetworkTables methods
        StructArrayEntry<Pose3d> entry = NetworkTableInstance.getDefault().getStructArrayTopic(key, Pose3d.struct).getEntry(new Pose3d[]{});
        // Store the entry in this instance, so it can be retrieved next time
        entries.put(key, entry);

        return entry;
    }

    /**
     * Publishes multiple poses to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param poses The poses to publish.
     */
    public static void publish(String key, Pose3d[] poses) {
        getEntry(key).set(poses);
    }

    /**
     * Publishes multiple poses to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param poses The poses to publish.
     */
    public static void publish(String key, Pose2d[] poses) {
        publish(key, Arrays.stream(poses).map(Pose3d::new).toArray(Pose3d[]::new));
    }

    /**
     * Publishes a single pose to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param pose The pose to publish.
     */
    public static void publish(String key, Pose3d pose) {
        publish(key, new Pose3d[]{pose});
    }

    /**
     * Publishes a single pose to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param pose The pose to publish.
     */
    public static void publish(String key, Pose2d pose) {
        publish(key, new Pose3d(pose));
    }

    /**
     * Publishes multiple translations to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param translations The translations to publish.
     */
    public static void publish(String key, Translation3d[] translations) {
        publish(key, Arrays.stream(translations).map(PoseUtils::translation3DToPose3D).toArray(Pose3d[]::new));
    }

    /**
     * Publishes multiple translations to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param translations The translations to publish.
     */
    public static void publish(String key, Translation2d[] translations) {
        publish(key, Arrays.stream(translations).map(PoseUtils::translation2DToPose2D).toArray(Pose2d[]::new));
    }

    /**
     * Publishes a single translation to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param translation The translation to publish.
     */
    public static void publish(String key, Translation3d translation) {
        publish(key, PoseUtils.translation3DToPose3D(translation));
    }

    /**
     * Publishes a single translation to the entry associated with the key.
     *
     * @param key The key to publish to.
     * @param translation The translation to publish.
     */
    public static void publish(String key, Translation2d translation) {
        publish(key, PoseUtils.translation2DToPose2D(translation));
    }
}
