package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.EnumSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Global constants for the robot */
public class RobotConstants {

  /** Number of robot periodic calls per second */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds */
  public static final Time PERIODIC_DURATION = Seconds.of(1.0 / PERIODIC_RATE);

  /** Number of fast peridic calls per second */
  public static final double FAST_PERIODIC_RATE = 100;

  /** Duration of each fast periodic call in seconds */
  public static final Time FAST_PERIODIC_DURATION = Seconds.of(1.0 / FAST_PERIODIC_RATE);

  /** Side lengths of robot chassis */
  public static final Distance CHASSIS_SIDE_LENGTH = Inches.of(24);

  // TODO Needs to be updated with exact CAD measurements
  /** Distances from the robot's center to the turret's center of rotation. */
  public static final Translation3d ROBOT_TO_TURRET = new Translation3d(
        Inches.of(-2.075),
        Inches.of(-3.9),
        Inches.of(14.939).plus(Meters.of(0.075))
  );

  /**
   * Gets the pose representing the turret's position and orientation in the robot's local frame.
   *
   * @param azimuth The turret's azimuth angle.
   * @return The turret pose in the robot's local frame.
   */
  public static Pose3d localTurretPose(Angle azimuth) {
    // TODO Create a factory method for creating yaw-only rotations
    Rotation3d yawRotation = new Rotation3d(Degrees.zero(), Degrees.zero(), azimuth);
    return new Pose3d(ROBOT_TO_TURRET, yawRotation);
  }

  /**
   * Gets the pose representing the turret's position and orientation in the world's global frame.
   *
   * @param robot The robot's pose in the world's global frame.
   * @param azimuth The turret's azimuth angle.
   * @return The turret pose in the world's global frame.
   */
  public static Pose3d globalTurretPose(Pose3d robot, Angle azimuth) {
    // TODO This calculation may be simplified by treating the turret's local pose as a transform
    Rotation3d yawRotation = new Rotation3d(Degrees.zero(), Degrees.zero(), azimuth);
    return new Pose3d(robot.getTranslation().plus(ROBOT_TO_TURRET.rotateBy(robot.getRotation())), robot.getRotation().plus(yawRotation));
  }

  /**
   * Gets the pose representing the turret's position and orientation in the world's global frame.
   *
   * @param robot The robot's pose in the world's global frame.
   * @param azimuth The turret's azimuth angle.
   * @return The turret pose in the world's global frame.
   */
  public static Pose3d globalTurretPose(Pose2d robot, Angle azimuth) {
    return globalTurretPose(new Pose3d(robot), azimuth);
  }

  /** Subsystems */
  public enum Subsystem {
    SWERVE,
    SHOOTER,
    AZIMUTH,
    HOOD,
    INTAKE,
    SPINDEXER,
    KICKER,
  }

  /** Enabled subsystems */
  public static final Set<Subsystem> ENABLED_SUBSYSTEMS = EnumSet.of(Subsystem.SWERVE, Subsystem.SHOOTER, Subsystem.AZIMUTH, Subsystem.HOOD, Subsystem.INTAKE, Subsystem.SPINDEXER, Subsystem.KICKER);
}
