package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Subsystem;
import frc.lib.sendables.SwerveDriveSendable;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.Gyroscope.GyroscopeValues;
import frc.lib.swerves.SwerveOutput;
import java.util.function.Supplier;

public class Drive extends Subsystem {

  private static Drive instance = null;

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private boolean hasSetPerspective = false;

  private PIDController yawPID;

  private final Gyroscope gyro;

  private final GyroscopeValues gyroValues = new GyroscopeValues();

  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }

    return instance;
  }

  private Drive() {
    swerve = DriveFactory.createSwerve();
    state = new SwerveDrivetrain.SwerveDriveState();
    field = new Field2d();
    gyro = DriveFactory.createGyroscope();

    yawPID = new PIDController(6, 0.0, 0.0);
    yawPID.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add("Field", field);
    tab.add("States", new SwerveDriveSendable(() -> state.ModuleStates, () -> this.getPose().getRotation()));
    tab.add("Targets", new SwerveDriveSendable(() -> state.ModuleTargets, () -> this.getPose().getRotation()));
  }

  @Override
  public void periodic() {
    state = swerve.getState();
    field.setRobotPose(state.Pose);
    gyro.periodic();

    // NOTE This was taken from the generated project, unsure if it is needed
    trySettingPerspective();
  }

  private void trySettingPerspective() {
    if (!hasSetPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(
        allianceColor -> {
          swerve.setOperatorPerspectiveForward(
            allianceColor == DriverStation.Alliance.Red
              ? Rotation2d.k180deg
              : Rotation2d.kZero);
            hasSetPerspective = true;
        }
      );
    }
  }

  /**
   * Takes a ChassisSpeeds object as input, and a target chassis rotation, and changes the angular velocity of
   * the ChassisSpeeds object to turn the robot towards the target rotation
   * 
   * @param speeds ChassisSpeeds object to modify
   * @param rotation Target chassis rotation
   * @return Modified ChassisSpeeds object
   */
  public ChassisSpeeds turnTowards(ChassisSpeeds speeds, Rotation2d rotation) {
    AngularVelocity angularVelocity = RotationsPerSecond.of(yawPID.calculate(state.Pose.getRotation().getRotations(), rotation.getRotations()));

    SmartDashboard.putNumber("error", Math.abs(rotation.getRotations()-state.Pose.getRotation().getRotations()));

    return new ChassisSpeeds(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      angularVelocity.in(RadiansPerSecond));
  }

  public ChassisSpeeds turnTowardsController(ChassisSpeeds speeds, CommandXboxController controller) {
    double x = MathUtil.applyDeadband(-controller.getRightY(), 0.1);
    double y = MathUtil.applyDeadband(-controller.getRightX(), 0.1);
    Rotation2d targetRotation = state.Pose.getRotation();
    if (!(x == 0 && y == 0)) {
      targetRotation = new Rotation2d(x, y);
      SmartDashboard.putNumber("targetRotation", targetRotation.getRotations());
    }

    return turnTowards(speeds, targetRotation);
  }

  public ChassisSpeeds turnTowardsTranslation(ChassisSpeeds speeds, Translation2d translation) {
    Translation2d robotRelative = translation.minus(state.Pose.getTranslation());

    return turnTowards(speeds, robotRelative.getAngle());
  }

  public Pose2d getPose() {
    return state.Pose;
  }

  public void resetPose(Pose2d newPose) {
    swerve.resetPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(state.Speeds, state.Pose.getRotation());
  }

  public SwerveDriveState getState() {
    return state;
  }

  public GyroscopeValues getGyroValues() {
    gyro.getUpdatedVals(gyroValues);
    return gyroValues;
  }

  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  public void setRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    swerve.setControl(robotCentric.withVelocityX(robotRelativeSpeeds.vxMetersPerSecond).withVelocityY(robotRelativeSpeeds.vyMetersPerSecond).withRotationalRate(robotRelativeSpeeds.omegaRadiansPerSecond));
  }

  public Command drive(Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    return this.run(() -> {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

      swerve.setControl(request
        .withVelocityX(fieldSpeeds.vxMetersPerSecond)
        .withVelocityY(fieldSpeeds.vyMetersPerSecond)
        .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond));
    });
  }

  public Command cross() {
    SwerveRequest.SwerveDriveBrake req = new SwerveRequest.SwerveDriveBrake();
    return run(() -> {
      swerve.setControl(req);
    });
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    swerve.addVisionMeasurement(pose, timestampSeconds);
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    swerve.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }
}