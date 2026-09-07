package frc.lib.motors.talonfxoutput;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.motors.MotorValues;

/**
 * Interface that wraps TalonFX hardware
 * Takes a Phoenix6 control request as input, and returns motor values back, whether
 * there's 1 motor, 2 motors, or simulated motors
 */
public interface TalonFXOutput {
  
  /**
   * Sets control of the TalonFX controlled device
   * 
   * @param request Phoenix6 control request
   */
  public void setControl(ControlRequest request);

  /**
   * Sets the position of the TalonFX controlled device
   * 
   * @param newPosition new position
   */
  public void setPosition(Angle newPosition);

  /**
   * Updates values stored in a MotorValues class and updates sim (if there is a sim)
   *
   * @param values values class to update
   * @param dt delta time to allow for sim calculation of values or values that require dt to be calculated
   */
  public void updateValues(MotorValues values, Time dt);

  /**
   * Configures motor hardware
   * 
   * @return true if configuration was successful
   */
  public boolean configure();
}
