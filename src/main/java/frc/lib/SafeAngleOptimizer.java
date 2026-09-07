package frc.lib;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/** 
 * Class used to optimize angle setpoints (instead of changing from 170 to -170, change 
 * from 170 to 190), within a certain safe range of setpoints
 * 
 * For example, with a safe angle range of (-540, 540):
 * When at position 170 and given setpoint -170, move to 190
 * When at position 520 and given setpoint -20, move back a rotation to 340
 */
public class SafeAngleOptimizer {

  /** Minimum safe setpoint in rotations */
  private double minAngle;

  /** Maximum safe setpoint in rotations */
  private double maxAngle;

  /** Center of the range of safe angles in rotations */
  private double center;

  /**
   * Constructs a SafeAngleOptimizer object with a minimum and maximum safe angle
   * At the moment, this wraps assuming 360 degree angles are coterminal
   * 
   * @param minAngle Minimum safe angle
   * @param maxAngle Maximum safe angle
   */
  public SafeAngleOptimizer(Angle minAngle, Angle maxAngle) {
    center = maxAngle.minus(minAngle).div(2).in(Rotations);
    this.minAngle = minAngle.in(Rotations);
    this.maxAngle = maxAngle.in(Rotations);
  }

  /**
   * Given a setpoint, returns a new coterminal setpoint that minimizes distance from the 
   * previous setpoint without rotating past safe limits
   * 
   * @param currentSetpoint Current setpoint
   * @param newSetpoint New setpoint to optimize and
   * @return Optimized setpoint
   */
  public Angle optimizeSetpoint(Angle currentSetpoint, Angle newSetpoint) {
    double newSetpointRotations = newSetpoint.in(Rotations);
    double currentSetpointRotations = currentSetpoint.in(Rotations);

    double distance = currentSetpointRotations - newSetpointRotations; // distance from new setpoint to current angle
    double lowEquivalent = newSetpointRotations + Math.floor(distance);
    double highEquivalent = newSetpointRotations + Math.ceil(distance);

    double lowDistance = Math.abs(currentSetpointRotations - lowEquivalent);
    double highDistance = Math.abs(currentSetpointRotations - highEquivalent);

    if (highDistance < lowDistance) {
      if (highEquivalent > maxAngle) {     
        // if higher equivalent is past max angle use lower anyways
        return Rotations.of(lowEquivalent);
      }
      // otherwise use the closer high equivalent
      return Rotations.of(highEquivalent);

    } else {
      if (lowEquivalent < minAngle) {      
        // if lower equivalent is past min angle use higher anyways
        return Rotations.of(highEquivalent);
      }
      // otherwise use the closer low equivalent
      return Rotations.of(lowEquivalent);
    }
  }

  /**
   * Gets the given angle's distance from the edges of the range of safe angles
   * 
   * @param angle angle to check
   * @return The given angle's distance from the edges of the range of safe angles
   * Negative values returned measure how far past the edges of the safe range the angle is
   */
  public Angle getCushion(Angle angle) {
    double angleRotations = angle.in(Rotations);
    return Rotations.of(Math.min(maxAngle-angleRotations, angleRotations-minAngle));
  }

  public Angle getMaxAngle() {
    return Rotations.of(maxAngle);
  }

  public Angle getMinAngle() {
    return Rotations.of(minAngle);
  }

  public Angle getCenter() {
    return Rotations.of(center);
  }
}
