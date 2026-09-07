// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveSpeedsUtils;
import frc.robot.hood.Hood;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterTester;
import frc.robot.spindexer.Spindexer;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureTrigger;
import frc.robot.turret.Turret;

/** Robot container */
public class RobotContainer {

  /** Robot container singleton */
  private static RobotContainer instance = null;

  /** Driver controller */
  private final CommandXboxController driverController;

  /** Operator controller */
  private final CommandXboxController operatorController;

  /** Multithreader */
  private final Multithreader multithreader;

  /** Drive */
  private final Drive drive;

  /** Shooter */
  private final Shooter shooter;

  /** Shooter tester */
  private final ShooterTester shooterTester;

  /** Azimuth */
  private final Azimuth azimuth;

  /** Hood */
  private final Hood hood;

  /** Turret */
  private final Turret turret;

  /** Intake */
  private final Intake intake;

  /** Spindexer */
  private final Spindexer spindexer;

  /** Kicker */
  private final Kicker kicker;

  /** Superstrcture */
  private final Superstructure superstructure;

  /** Autonomous handler */
  private final AutonomousHandler autoHandler;

  /**
   * Gets robot container instance
   * 
   * @return robot container instance
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  /** Initializes robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    multithreader = Multithreader.getInstance();
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();
    shooterTester = ShooterTester.getInstance();
    azimuth = Azimuth.getInstance();
    hood = Hood.getInstance();
    turret = Turret.getInstance();
    intake = Intake.getInstance();
    spindexer = Spindexer.getInstance();
    kicker = Kicker.getInstance();
    superstructure = Superstructure.getInstance();
    autoHandler = AutonomousHandler.getInstance();

    multithreader.start();

    Telemetry.initializeTabs();
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    if (TuningConstants.ENABLE_SWERVE_LIMITS) {
      drive.setDefaultCommand(drive.drive(() -> {
        ChassisSpeeds driverSpeeds = DriveSpeedsUtils.fromController(
                driverController,
                TuningConstants.SWERVE_VELOCITY_LIMIT,
                TuningConstants.SWERVE_ANGULAR_VELOCITY_LIMIT,
                0.1);
        return DriveSpeedsUtils.limitAcceleration(driverSpeeds, TuningConstants.SWERVE_LINEAR_ACCEL_LIMIT, TuningConstants.SWERVE_ANGULAR_ACCEL_LIMIT);
      }));
    } else {
      drive.setDefaultCommand(drive.drive(() -> {
        return DriveSpeedsUtils.fromController(
                driverController,
                MetersPerSecond.of(2.75),
                RotationsPerSecond.of(0.75),
                0.1);
      }));
    }
  }

  private boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }

  private void configureBindings() {
    driverController.a().onTrue(superstructure.faceHub());
    driverController.b().onTrue(superstructure.stow());
    driverController.x().onTrue(superstructure.intake());
    driverController.y().onTrue(superstructure.score());
    driverController.leftBumper().onTrue(superstructure.init());

    driverController.povUp().onTrue(superstructure.pass(
      new SuperstructureTrigger(() -> driverController.leftTrigger().getAsBoolean()),
      new SuperstructureTrigger(() -> driverController.rightTrigger().getAsBoolean())));

    driverController.povDown().onTrue(superstructure.passFar(
      new SuperstructureTrigger(() -> driverController.leftTrigger().getAsBoolean()),
      new SuperstructureTrigger(() -> driverController.rightTrigger().getAsBoolean())));

    /**
    operatorController.povDown().onTrue(superstructure.feed());
    operatorController.povRight().onTrue(Commands.runOnce(() -> shooter.setSetpoint(RotationsPerSecond.of(15))));
    operatorController.rightBumper().whileTrue(Commands.run(() -> shooter.setSetpoint(shooter.getSetpoint().plus(RotationsPerSecond.of(0.1)))));
    operatorController.leftBumper().whileTrue(Commands.run(() -> shooter.setSetpoint(shooter.getSetpoint().minus(RotationsPerSecond.of(0.1)))));
    operatorController.rightTrigger().whileTrue(Commands.run(() -> hood.setSetpoint(hood.getSetpoint().plus(Rotations.of(0.001)))));
    operatorController.leftTrigger().whileTrue(Commands.run(() -> hood.setSetpoint(hood.getSetpoint().minus(Rotations.of(0.001)))));
    */
    operatorController.x().onTrue(Commands.runOnce(() -> superstructure.getCurrentCommand().cancel()));
    operatorController.b().onTrue(Commands.runOnce(() -> turret.changeShotVelocity(RotationsPerSecond.of(1))));
    operatorController.a().onTrue(Commands.runOnce(() -> turret.changeShotVelocity(RotationsPerSecond.of(-1))));
    operatorController.y().onTrue(Commands.runOnce(() -> turret.setExtraShotVelocity(RotationsPerSecond.of(0))));

    operatorController.leftBumper().onTrue(Commands.runOnce(() -> azimuth.resetPosition(Rotations.of(0.25))));
  }

  public Command getAutonomousCommand() {
    return autoHandler.getAutonomousCommand();
  }
}
