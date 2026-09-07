package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;

/** Class with static methods to help with getting chassis speeds to drive the drive subsystem */
public class DriveSpeedsUtils {

  /** Drive subsystem reference */
  private static Drive drive = Drive.getInstance();

  /**
   * Gets field relative chassis speeds from controller input with some max linear and angular velocity
   * 
   * @param controller Controller to grab inputs from
   * @param maxLinearVelocity Maximum desired linear velocity
   * @param maxAngularVelocity Maximum desired angular velocity
   * @param deadzone Controller deadzone (uses deadzone of 0.1 by default)
   * @return Field relative chassis speeds constructed from controller input with some max linear and angular velocity
   */
  public static ChassisSpeeds fromController(CommandXboxController controller, LinearVelocity maxLinearVelocity, AngularVelocity maxAngularVelocity, double deadzone) {
    Translation2d translation = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    double deadzoneSlope = 1/(1-deadzone);
    double deadzonedTranslationMagnitude = Math.max(0, deadzoneSlope * translation.getNorm() - deadzoneSlope * deadzone);
    SmartDashboard.putNumber("DEADZONED VELOCITY MAG", deadzonedTranslationMagnitude);
    Translation2d deadzonedTranslation = translation.div(translation.getNorm() / deadzonedTranslationMagnitude);

    LinearVelocity vx = maxLinearVelocity.times(deadzonedTranslation.getX());
    LinearVelocity vy = maxLinearVelocity.times(deadzonedTranslation.getY());

    double rotationMagnitude = -controller.getRightX();
    double deadzonedRotationMagnitude = Math.copySign(Math.max(0, deadzoneSlope * Math.abs(rotationMagnitude) - deadzoneSlope * deadzone), rotationMagnitude);

    AngularVelocity vomega = maxAngularVelocity.times(deadzonedRotationMagnitude);

    return new ChassisSpeeds(vx, vy, vomega);
  }

  /**
   * Gets field relative chassis speeds from controller input with some max linear and angular velocity
   * 
   * @param controller Controller to grab inputs from
   * @param maxLinearVelocity Maximum desired linear velocity
   * @param maxAngularVelocity Maximum desired angular velocity
   * @return Field relative chassis speeds constructed from controller input with some max linear and angular velocity
   */
  public static ChassisSpeeds fromController(CommandXboxController controller, LinearVelocity maxLinearVelocity, AngularVelocity maxAngularVelocity) {
    return fromController(controller, maxLinearVelocity, maxAngularVelocity, 0.1);
  }

  /**
   * Returns chassis speeds with limited linear and angular acceleration relative to the robot's current speeds
   * 
   * @param speeds Chassis speeds to be limited
   * @param maxLinearAcceleration Maximum desired linear acceleration
   * @param maxAngularAcceleration Maximum desired angular acceleration
   * @param dt Time passed between calls (uses RobotConstants.PERIODIC_DURATION by default)
   * @return chassis speeds iwth limited linear and angular acceleration relative to the robot's current speeds
   */
  public static ChassisSpeeds limitAcceleration(ChassisSpeeds speeds, LinearAcceleration maxLinearAcceleration, AngularAcceleration maxAngularAcceleration, Time dt) {
    ChassisSpeeds botSpeeds = drive.getState().Speeds;

    Translation2d requestedVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Translation2d botVelocity = new Translation2d(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond);
    Translation2d requestedAcceleration = requestedVelocity.minus(botVelocity).div(dt.in(Seconds));
    System.out.println(requestedAcceleration.getNorm());
    if (requestedAcceleration.getNorm() > maxLinearAcceleration.in(MetersPerSecondPerSecond)) {
      requestedAcceleration = requestedAcceleration.div(requestedAcceleration.getNorm()).times(maxLinearAcceleration.in(MetersPerSecondPerSecond));
    }
    Translation2d accelerationLimitedVelocity = botVelocity.plus(requestedAcceleration.times(dt.in(Seconds)));

    double requestedAngularAcceleration = (speeds.omegaRadiansPerSecond - botSpeeds.omegaRadiansPerSecond) / dt.in(Seconds);
    double accelerationLimitedAngularVelocity = botSpeeds.omegaRadiansPerSecond + (MathUtil.clamp(requestedAngularAcceleration, -maxAngularAcceleration.in(RadiansPerSecondPerSecond), maxAngularAcceleration.in(RadiansPerSecondPerSecond)) * dt.in(Seconds));

    return new ChassisSpeeds(accelerationLimitedVelocity.getX(), accelerationLimitedVelocity.getY(), accelerationLimitedAngularVelocity);
  }

  /**
   * Returns chassis speeds with limited linear and angular acceleration relative to the robot's current speeds
   * 
   * @param speeds Chassis speeds to be limited
   * @param maxLinearAcceleration Maximum desired linear acceleration
   * @param maxAngularAcceleration Maximum desired angular acceleration
   * @return chassis speeds iwth limited linear and angular acceleration relative to the robot's current speeds
   */
  public static ChassisSpeeds limitAcceleration(ChassisSpeeds speeds, LinearAcceleration maxLinearAcceleration, AngularAcceleration maxAngularAcceleration) {
    return limitAcceleration(speeds, maxLinearAcceleration, maxAngularAcceleration, RobotConstants.PERIODIC_DURATION);
  }
}
