package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureTrigger;

public class AutonomousHandler {
  
  /** AutonomousHandler instance */
  private static AutonomousHandler instance = null;

  /** Drive reference */
  private final Drive drive = Drive.getInstance();

  /** Superstructure reference */
  private final Superstructure superstructure = Superstructure.getInstance();

  /** Azimuth reference */
  private final Azimuth azimuth = Azimuth.getInstance();

  /** Used in dashboard to pick what autonomous routine to run */
  private final SendableChooser<Command> autoChooser;

  public static AutonomousHandler getInstance() {
    if (instance == null) {
      instance = new AutonomousHandler();
    }

    return instance;
  }

  private AutonomousHandler() {
    // Configure AutoBuilder
    RobotConfig robotConfig = null;

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportWarning("Couldn't get robot config from GUI settings", true);
    }

    AutoBuilder.configure(
      drive::getPose, 
      drive::resetPose, 
      drive::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> drive.setRobotRelativeSpeeds(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(10, 0, 0), 
        new PIDConstants(7, 0, 0)), 
      robotConfig, 
      () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent(),
      drive);

    // Set up named commands
    NamedCommands.registerCommand("FaceHub", superstructure.faceHub().asProxy());
    NamedCommands.registerCommand("Intake", superstructure.intake().asProxy());
    NamedCommands.registerCommand("Score1", superstructure.score().asProxy().andThen(Commands.waitSeconds(1)).andThen(superstructure.faceHub().asProxy()));
    NamedCommands.registerCommand("Score2", superstructure.score().asProxy().andThen(Commands.waitSeconds(2)).andThen(superstructure.faceHub().asProxy()));
    NamedCommands.registerCommand("Score3", superstructure.score().asProxy().andThen(Commands.waitSeconds(3)).andThen(superstructure.faceHub().asProxy()));
    NamedCommands.registerCommand("Score5", superstructure.score().asProxy().andThen(Commands.waitSeconds(5)).andThen(superstructure.faceHub().asProxy()));
    NamedCommands.registerCommand("Score", superstructure.score().asProxy());
    NamedCommands.registerCommand("Pass", superstructure.pass(
      new SuperstructureTrigger(() -> true), 
      new SuperstructureTrigger(() -> azimuth.nearSetpoint(Rotations.of(0.1)))
    ).asProxy());

    new EventTrigger("Intake").onTrue(superstructure.intake().asProxy());
    
    // Publish auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
