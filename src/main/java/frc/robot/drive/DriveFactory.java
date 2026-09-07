package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import frc.lib.CAN;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.sensors.GyroscopeSim;
import frc.lib.swerves.PhoenixSwerve;
import frc.lib.swerves.SwerveOutput;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DriveFactory {

  public static SwerveOutput createSwerve() {
      return new PhoenixSwerve(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  public static Gyroscope createGyroscope() {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.SWERVE)) {
      return new GyroscopePigeon2(new CAN(0, "swerve"));
    }

    return new GyroscopeSim(() -> RadiansPerSecond.of(Drive.getInstance().getState().Speeds.omegaRadiansPerSecond));
  }
}
