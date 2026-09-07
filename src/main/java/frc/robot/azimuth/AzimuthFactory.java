package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.motoroutput.LossyMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.lib.motors.motoroutput.MotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutputTalonFXCANcoder;
import frc.lib.motors.talonfxoutput.DiscreteTalonFXOutputSim;
import frc.lib.motors.talonfxoutput.TalonFXOutput;
import frc.lib.motors.talonfxoutput.TalonFXOutput1;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.sensors.GyroscopeSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.drive.Drive;

/** Creates azimuth hardware abstractions */
public class AzimuthFactory {
  
  public static TalonFXOutput createAzimuthMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AZIMUTH)) {
      return new TalonFXOutput1(config, new CAN(0));
    }

    return new DiscreteTalonFXOutputSim();
  }
}
