package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import java.io.Externalizable;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import javax.lang.model.util.Elements.Origin;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.lib.PosePublisher;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.TuningConstants;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.shooter.Shooter;

/** Turret subsystem */
public class Turret extends MultithreadedSubsystem {

  /** Turret subsystem singleton */
  private static Turret instance = null;

  /** Azimuth subsystem reference */
  private final Azimuth azimuth;

  /** Hood subsystem reference */
  private final Hood hood;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /** Drive subsystem reference */
  private final Drive drive;

  /** Current turret state */
  private TurretState state;

  /** Estimated turret pose */
  private volatile Pose2d turretPose;

  // State specific control variables

  /** Robot pose used by targetHubFromPose */
  private Pose2d manualRobotPose;

  /** Used when targetting hub and shots are falling short for some reason */
  private final MutAngularVelocity extraShotVelocity = RotationsPerSecond.mutable(0.0); // TODO: I HATE THIS

  // vision sim
  VisionSystemSim visionSim = new VisionSystemSim("main");

  // The PhotonCamera used in the real robot code.
  PhotonCamera camera = new PhotonCamera("PhotonVisionCamera");

  // The simulation of this camera. Its values used in real robot code will be updated.
  PhotonCameraSim cameraSim;

  // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
  public static Translation3d robotToCameraTrl = new Translation3d(Inches.of(5.584), Inches.of(2.145), Inches.of(17.5));
  // and pitched 15 degrees up.
  public static Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
  public static Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

  // The layout of AprilTags which we want to add to the vision system
  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Used for vision-only targeting
  double tx;
  double distance;
  int bestFiducialID = 10;
  boolean hasFiducial = false;
  boolean targetVisible = false;
  Angle targetYaw = Degrees.zero();
  Distance targetRange = Meters.zero();

  /**
   * Gets turret subsystem instance
   * 
   * @return turret subsystem instance
   */
  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }

    return instance;
  }

  /** Turret subsystem constructor */
  private Turret() {
    azimuth = Azimuth.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    drive = Drive.getInstance();

    state = TurretState.STOW;


    /* 
    //camera sim setup
    SimCameraProperties cameraProp = new SimCameraProperties();
    // A 640 x 480 camera with a 100 degree diagonal FOV.322
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(99.3));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    */
    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    //visionSim.addCamera(cameraSim, robotToCamera);

    //visionSim.addAprilTags(tagLayout);
    //SmartDashboard.putData(visionSim.getDebugField());
    }

  @Override 
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    tab.addString("State", () -> state.name());
    tab.addBoolean("At target state", () -> atTargetState());
  }

    public List<Transform3d> getOriginToCameraTransforms(){
        
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        List<Optional<Transform3d>> transforms = results.stream().map(this::getMultiTagResult).toList();
        List<Transform3d> validTransforms = transforms.stream().filter(Optional::isPresent).map(Optional::get).toList();

        return validTransforms;
    }

    public Optional<Transform3d> getMultiTagResult(PhotonPipelineResult result){
       if (result.hasTargets()){
            var bestTarget = result.getMultiTagResult();

            if (bestTarget.isEmpty()){
              return Optional.empty();
            }
            
            Transform3d originToCamera = bestTarget.get().estimatedPose.best;

            return Optional.of(originToCamera);
        }
      return Optional.empty();
    }



    public Pose3d getRobotPose(Transform3d originToCamera, Transform3d cameraToRobot){

        Transform3d robotInField = originToCamera.plus(cameraToRobot);

        return new Pose3d().plus(robotInField);
    }

    public Pose2d get2dRobotPose(Pose3d visionPose){

      Pose2d newPose2d = visionPose.toPose2d();
      return newPose2d;
    }

    public void setBestAllianceFiducial(Alliance alliance){
      if (alliance == Alliance.Red){
        bestFiducialID = 10;
      } else if (alliance == Alliance.Blue){
        bestFiducialID = 26;
      }
    }


  @Override
  public void periodic() {
    if (Robot.isSimulation()){ 
    Pose2d robot = Drive.getInstance().getPose();
    PosePublisher.publish("Turret (Local)", RobotConstants.localTurretPose(azimuth.localPosition()));
    PosePublisher.publish("Turret (Global)", RobotConstants.globalTurretPose(robot, azimuth.localPosition()));
    
    visionSim.update(drive.getPose());
    visionSim.adjustCamera(cameraSim, robotToCamera);

    }


    //TODO: Maybe try moving this to fastPeriodic() if it isn't too much of a performance hit
    Translation2d translationToHub = TurretTargetsSupplier.projectedAllianceHub().minus(turretPose.getTranslation());
    SmartDashboard.putNumber("Turret distance (meters)", translationToHub.getNorm());

    PosePublisher.publish("Estimated turret pose", turretPose);
    
    Rotation3d turretRotation = new Rotation3d(Radians.zero(), Radians.zero(), azimuth.getValues().position);
    robotToCamera = new Transform3d(
      robotToCameraTrl.rotateBy(turretRotation),
      robotToCameraRot.rotateBy(turretRotation)
      );

    var multiTagRobotPoses = getOriginToCameraTransforms().stream().map((t) -> getRobotPose(t, robotToCamera.inverse())).toArray(Pose3d[]::new);
    PosePublisher.publish("RobotPose", multiTagRobotPoses);

    for(Pose3d robotPose:multiTagRobotPoses){
      drive.addVisionMeasurement(robotPose.toPose2d(), Timer.getFPGATimestamp(), VecBuilder.fill(1.5, 1.5, 9999));
    }

    if (Robot.isReal()){
        // Read in relevant data from the Camera

        DriverStation.getAlliance().ifPresent(this::setBestAllianceFiducial);
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == bestFiducialID) {
                      // Found Tag 7, record its information
                      targetYaw = Degrees.of(target.getYaw());
                      targetVisible = true;

                      targetRange =
                                Meters.of(PhotonUtils.calculateDistanceToTargetMeters(
                                        0.735102, // Measured with a tape measure, or in CAD.
                                        1.30048, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-15.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch())));

                    }
                }
            }
        }
    }
    



  }

  @Override
  public void fastPeriodic() {
    switch (state) {
      case STOW:
        azimuth.setSetpoint(Rotations.of(0.25));
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case ALLOW_EXTERNAL_CONTROL:
        break;
      case ALLOW_EXTERNAL_CONTROL_FACING_HUB:
        faceHub(targetYaw);
        break;
      case FACE_HUB:
        faceHub(targetYaw);
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case TARGET_HUB:
        faceHub(targetYaw);
        targetHub(targetRange);
        break;
      case PASS:
        faceAllianceWall(turretPose);
        shooter.setSetpoint(RotationsPerSecond.of(45));
        hood.setSetpoint(Rotations.of(0.06));
        break;
      case PASS_FAR:
        faceAllianceWall(turretPose);
        shooter.setSetpoint(RotationsPerSecond.of(60));
        hood.setSetpoint(Rotations.of(0.075));
        break;
      case PASS_SOTM:
        SwerveDriveState driveState = Drive.getInstance().getState();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(driveState.Speeds, driveState.Pose.getRotation());
        faceAllianceWallSOTM(turretPose, new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond));
        if (TuningConstants.ENABLE_POTM_CHANGES) {
          shooter.setSetpoint(TuningConstants.POTM_FLYWHEEL_SETPOINT);
          hood.setSetpoint(TuningConstants.POTM_HOOD_SETPOINT);
        } else {
          shooter.setSetpoint(RotationsPerSecond.of(50));
          hood.setSetpoint(Rotations.of(0.07));
        }
        break;
      case TARGET_HUB_FROM_POSE:
        Pose2d manualTurretPose = RobotConstants.globalTurretPose(manualRobotPose, azimuth.getValues().position).toPose2d();
        faceHub(targetYaw);
        targetHub(targetRange);
        break;
    }
  }

  public boolean atTargetState() {
    switch (state) {
      case STOW:
        return azimuth.nearSetpoint(Rotations.of(0.01));
      case FACE_HUB:
        return azimuth.nearSetpoint(Rotations.of(0.1));
      case TARGET_HUB:
        return azimuth.nearSetpoint(Rotations.of(0.075))
            && shooter.nearSetpoint(RotationsPerSecond.of(4))
            && hood.nearSetpoint(Rotations.of(0.1));
      case PASS, PASS_FAR:
        return azimuth.nearSetpoint(Rotations.of(0.1))
            && shooter.nearSetpoint(RotationsPerSecond.of(4))
            && hood.nearSetpoint(Rotations.of(0.1));
      case PASS_SOTM:
        return azimuth.nearSetpoint(Rotations.of(0.2))
            && shooter.nearSetpoint(RotationsPerSecond.of(4))
            && hood.nearSetpoint(Rotations.of(0.1));
      default:
        return true;
    }
  }

  /** 
   * Turns the azimuth to face your alliance hub given an estimated turret pose
   * 
   * @param turretPose estimated turret pose
   */
  private void faceHub(Angle yaw) {
    azimuth.setSetpoint(azimuth.getValues().position.plus(yaw));





    /*  Old Stuff for not at Robots on Fire
    Translation2d translationToHub = TurretTargetsSupplier.projectedAllianceHub().minus(turretPose.getTranslation());
    Rotation2d rotationError = translationToHub.getAngle().minus(turretPose.getRotation());

    SmartDashboard.putNumber("Turret distance (meters)", translationToHub.getNorm()); // getNorm does return distance in meters it isn't documented but translations are in meters

    azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
    */
  }

  private void faceAllianceWall(Pose2d turretPose) {
    Rotation2d rotationError = TurretTargetsSupplier.faceAllianceWall().minus(turretPose.getRotation());

    azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
  }

  private void faceAllianceWallSOTM(Pose2d turretPose, Translation2d turretVelocity) {
    if (TuningConstants.ENABLE_POTM_CHANGES) {
      Translation2d shotVelocity = new Translation2d(TuningConstants.POTM_FUEL_EXIT_VELOCITY_FUDGE.in(MetersPerSecond), turretPose.getRotation()).plus(turretVelocity);
      Rotation2d rotationError = TurretTargetsSupplier.faceAllianceWall().minus(shotVelocity.getAngle().times(TuningConstants.POTM_ANGLE_OFFSET_SCALAR_FUDGE));

      azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
    } else {
      Translation2d shotVelocity = new Translation2d(4, turretPose.getRotation()).plus(turretVelocity);
      Rotation2d rotationError = TurretTargetsSupplier.faceAllianceWall().minus(shotVelocity.getAngle());

      azimuth.setSetpoint(azimuth.getValues().position.plus(rotationError.getMeasure()));
    }
  }

  private void targetHub(Distance targetRange) {
    Angle hoodSetpoint = TurretTargeter.targetHubHood(targetRange);
    AngularVelocity shooterSetpoint = TurretTargeter.targetHubShooter(targetRange);

    hood.setSetpoint(hoodSetpoint);
    shooter.setSetpoint(shooterSetpoint.plus(extraShotVelocity)); // TODO: I HATE THIS
  }

  /**
   * Stows the turret and waits until fully stowed
   * 
   * @return a command that stows the turret and waits until fully stowed
   */
  public Command stow() {
    return Commands.runOnce(() -> {
      state = TurretState.STOW;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Allows external control of azimuth, shooter, and hood using their setSetpoint methods
   * 
   * @return a command that allows external control of azimuth, shooter, and hood using their setSetpoint methods
   */
  public Command allowExternalControl() {
    return Commands.runOnce(() -> {
      state = TurretState.ALLOW_EXTERNAL_CONTROL;
    }, this)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Allows external control of shooter and hood using their setSetpoint methods while looking at the hub with azimuth
   * 
   * @return a command that allows external control of shooter and hood using their setSetpoint methods while looking at the hub with azimuth
   */
  public Command allowExternalControlFacingHub() {
    return Commands.runOnce(() -> {
      state = TurretState.ALLOW_EXTERNAL_CONTROL_FACING_HUB;
    }, this, azimuth)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Faces the azimuth at the alliance hub and waits until facing the hub
   * 
   * @return a command that faces the azimuth at the aliance hub and waits until facing the hub
   */
  public Command faceHub() {
    return Commands.runOnce(() -> {
      state = TurretState.FACE_HUB;
    }, this, azimuth)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  /**
   * Targets the alliance hub with the azimuth, shooter, and hood, and waits until targetting accurately
   * 
   * @return a command that targets the alliance hub with the azimuth, shooter, and hood, and waits until targetting accurately
   */
  public Command targetHub() {
    return Commands.runOnce(() -> {
      state = TurretState.TARGET_HUB;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public Command pass() {
    return Commands.runOnce(() -> {
      state = TurretState.PASS;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public Command passFar() {
    return Commands.runOnce(() -> {
      state = TurretState.PASS_FAR;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public Command passSOTM() {
    return Commands.runOnce(() -> {
      state = TurretState.PASS_SOTM;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState)); 
  }

  public Command targetHubFromPose(Pose2d robotPose) {
    return Commands.runOnce(() -> {
      state = TurretState.TARGET_HUB_FROM_POSE;
      manualRobotPose = robotPose;
    }, this, azimuth, hood, shooter)
    .andThen(Commands.waitUntil(this::atTargetState));
  }

  public void changeShotVelocity(AngularVelocity change) {
    extraShotVelocity.mut_plus(change);
  }

  public void setExtraShotVelocity(AngularVelocity newExtraShotVelocity) {
    extraShotVelocity.mut_replace(newExtraShotVelocity);
  }
}
