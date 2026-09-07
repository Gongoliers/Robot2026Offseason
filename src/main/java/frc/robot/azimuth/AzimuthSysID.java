package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

/** Class with methods to run sysid with azimuth */
public class AzimuthSysID {
  
  /** Azimuth subsystem reference */
  private static final Azimuth azimuth;

  /** Sysid config */
  private static final SysIdRoutine.Config config;

  /** Sysid mechanism */
  private static final SysIdRoutine.Mechanism mechanism;

  /** Sysid routine */
  private static final SysIdRoutine routine;

  /** Voltage out variable used for manual azimuth control */
  private static final MutVoltage voltageOut;

  static {
    azimuth = Azimuth.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(0.5),
      Volts.of(2),
      Seconds.of(5));

    mechanism = new SysIdRoutine.Mechanism(
      AzimuthSysID::setVoltage, 
      AzimuthSysID::logMotors, 
      azimuth);

    routine = new SysIdRoutine(config, mechanism);
  }

  private static void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private static void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = azimuth.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      azimuth.runAtVoltage(() -> voltageOut).asProxy(),
      routine.quasistatic(direction)
    );
  }

  public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      azimuth.runAtVoltage(() -> voltageOut).asProxy(),
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
      AzimuthSysID::sysIdQuasistatic, 
      AzimuthSysID::sysIdDynamic, 
      () -> azimuth.getValues().velocity.abs(RotationsPerSecond) < 0.1,
      azimuth);
  }
}
