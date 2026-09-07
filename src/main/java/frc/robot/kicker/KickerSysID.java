package frc.robot.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

public class KickerSysID {
  
  private static final Kicker kicker;

  private static final SysIdRoutine.Config config;

  private static final SysIdRoutine.Mechanism mechanism;

  private static final SysIdRoutine routine;

  private static final MutVoltage voltageOut;

  static {
    kicker = Kicker.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(0.4),
      Volts.of(3),
      Seconds.of(10));

    mechanism = new SysIdRoutine.Mechanism(
      KickerSysID::setVoltage, 
      KickerSysID::logMotors, 
      kicker);

    routine = new SysIdRoutine(config, mechanism);
  }

  private static void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private static void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = kicker.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      kicker.runAtVoltage(() -> voltageOut).asProxy(),
      routine.quasistatic(direction)
    );
  }

  public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      kicker.runAtVoltage(() -> voltageOut).asProxy(),
      routine.dynamic(direction)
    );
  }

  public static Command runFullSysId() {
    return new runFullSysId(
      KickerSysID::sysIdQuasistatic, 
      KickerSysID::sysIdDynamic, 
      () -> kicker.getValues().velocity.abs(RotationsPerSecond) < 0.1, 
      kicker);
  }
}
