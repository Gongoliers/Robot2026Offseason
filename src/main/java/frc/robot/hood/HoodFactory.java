package frc.robot.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.motoroutput.DiscreteMotorOutputSim;
import frc.lib.motors.motoroutput.LossyMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.lib.motors.motoroutput.MotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutputTalonFX;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/** Creates hood hardware abstractions */
public class HoodFactory {
  
  public static MotorOutput createHoodMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.HOOD)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(20));
    }

    // TODO: for now, use physically inaccurate discrete sim
    return new DiscreteMotorOutputSim();

    /*
    MotorOutputSim losslessSim = new MotorOutputSim(
      Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kA()),
      Volts.per(RotationsPerSecondPerSecond).ofNative(config.feedforwardControllerConfig().kA()),
      DCMotor.getKrakenX44(1));

    return new LossyMotorOutputSim(
      losslessSim, 
      Volts.of(config.feedforwardControllerConfig().kS()),
      (Angle position) -> Volts.of(Math.cos(position.in(Radians)) * config.feedforwardControllerConfig().kG()));
    */
  }
}
