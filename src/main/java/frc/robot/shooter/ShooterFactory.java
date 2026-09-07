package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.motoroutput.LossyMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.lib.motors.motoroutput.MotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutputTalonFX;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/** Creates shooter hardware abstractions */
public class ShooterFactory {
  
  public static MotorOutput createShooterMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.SHOOTER)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(10));
    }

    MotorOutputSim losslessSim = new MotorOutputSim(
      Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kV()),
      Volts.per(RotationsPerSecondPerSecond).ofNative(config.feedforwardControllerConfig().kA()),
      DCMotor.getKrakenX60(1));

    return new LossyMotorOutputSim(
      losslessSim, 
      Volts.of(config.feedforwardControllerConfig().kS()));
  }
}
