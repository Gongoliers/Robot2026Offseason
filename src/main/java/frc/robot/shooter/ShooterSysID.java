package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

/** Class with methods to run sysid and test the shooter */
public class ShooterSysID {

  /** Shooter subsystem reference */
  private static final Shooter shooter;

  /** Sysid config */
  private static final SysIdRoutine.Config config;

  /** Sysid mechanism */
  private static final SysIdRoutine.Mechanism mechanism;

  /** Sysid routine */
  private static final SysIdRoutine routine;

  /** Voltage out variable used for manual shooter control */
  private static final MutVoltage voltageOut;

  static {
    shooter = Shooter.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(1),
      Volts.of(7),
      Seconds.of(10));
    
    mechanism = new SysIdRoutine.Mechanism(
      ShooterSysID::setVoltage, 
      ShooterSysID::logMotors, 
      shooter);

    routine = new SysIdRoutine(config, mechanism);
  }

  private static void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private static void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = shooter.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut).asProxy(),
      routine.quasistatic(direction)
    );
  }

  public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut).asProxy(),
      routine.dynamic(direction)
    );
  }

  /**
   * Returns a command that runs quasistatic and dynamic sysid forwards and backwards
   * 
   * @return a command that runs quasistatic and dynamic sysid forwards and backwards
   */
  public static Command runFullSysId() {
    return new runFullSysId(
      ShooterSysID::sysIdQuasistatic, 
      ShooterSysID::sysIdDynamic, 
      () -> shooter.getValues().velocity.abs(RotationsPerSecond) < 0.1,
      shooter);
  }
}