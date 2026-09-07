package frc.robot.turret;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Class with static methods to get turret targets on the field */
public class TurretTargetsSupplier {
  
  /** Gets the 2d translation of your alliance hub */
  public static Translation2d projectedAllianceHub() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Translation2d(
        Inches.of(468.56),
        Inches.of(158.32));
    }

    return new Translation2d(
      Inches.of(181.56),
      Inches.of(158.32));
  }

  public static Rotation2d faceAllianceWall() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return Rotation2d.fromDegrees(0);
    }

    return Rotation2d.fromDegrees(180);
  }
}
