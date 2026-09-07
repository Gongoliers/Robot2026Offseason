package frc.robot.kicker;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.motoroutput.DiscreteMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.lib.motors.motoroutput.MotorOutputTalonFX;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class KickerFactory {
  
  public static MotorOutput createKickerMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.KICKER)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(30));
    }

    return new DiscreteMotorOutputSim();
  }
}
