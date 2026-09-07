package frc.lib;

import edu.wpi.first.math.geometry.*;

public class PoseUtils {

    public static Pose3d translation3DToPose3D(Translation3d translation) {
        return new Pose3d(translation, Rotation3d.kZero);
    }

    public static Pose2d translation2DToPose2D(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

}
