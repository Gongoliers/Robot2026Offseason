package frc.robot.spindexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.motoroutput.DiscreteMotorOutputSim;
import frc.lib.motors.motoroutput.LossyMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.lib.motors.motoroutput.MotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutputTalonFX;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class SpindexerFactory {
  
  public static MotorOutput createSpindexerMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.SPINDEXER)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(14));
    }

    return new DiscreteMotorOutputSim();

    /**
    MotorOutputSim losslessSim = new MotorOutputSim(
      Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kV()),
      Volts.per(RotationsPerSecondPerSecond).ofNative(config.feedforwardControllerConfig().kA()),
      DCMotor.getKrakenX60(1));

    return new LossyMotorOutputSim(
      losslessSim, 
      Volts.of(config.feedforwardControllerConfig().kS()));
    */
  }
}
