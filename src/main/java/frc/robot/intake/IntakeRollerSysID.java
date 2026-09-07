package frc.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

public class IntakeRollerSysID {
  
  private static final Intake intake;

  private static final SysIdRoutine.Config config;

  private static final SysIdRoutine.Mechanism mechanism;

  private static final SysIdRoutine routine;

  private static final MutVoltage voltageOut;

  static {
    intake = Intake.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(0.75),
      Volts.of(6),
      Seconds.of(10));

    mechanism = new SysIdRoutine.Mechanism(
      IntakeRollerSysID::setVoltage, 
      IntakeRollerSysID::logMotors, 
      intake);

    routine = new SysIdRoutine(config, mechanism);
  }

  private static void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private static void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = intake.getRollerValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      intake.runRollerAtVoltage(() -> voltageOut).asProxy(),
      routine.quasistatic(direction)
    );
  }

  public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      intake.runRollerAtVoltage(() -> voltageOut).asProxy(),
      routine.dynamic(direction)
    );
  }

  public static Command runFullSysId() {
    return new runFullSysId(
      IntakeRollerSysID::sysIdQuasistatic, 
      IntakeRollerSysID::sysIdDynamic, 
      () -> intake.getRollerValues().velocity.abs(RotationsPerSecond) < 0.1, 
      intake);
  }
}
