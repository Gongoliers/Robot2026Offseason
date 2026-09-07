package frc.robot.intake;

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

public class IntakeFactory {
  
  public static MotorOutput createPivotMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.INTAKE)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(12));
    }

    return new DiscreteMotorOutputSim();
  }

  public static MotorOutput createRollerMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.INTAKE)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(13));
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
