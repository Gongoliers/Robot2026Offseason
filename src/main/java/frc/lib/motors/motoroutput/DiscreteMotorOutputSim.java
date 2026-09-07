package frc.lib.motors.motoroutput;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import frc.lib.motors.MotorValues;

public class DiscreteMotorOutputSim implements MotorOutput {

  /** Output voltage set by setVoltage method */
  private MutVoltage outputVoltage;

  private Supplier<Angle> positionSupplier;

  private Supplier<AngularVelocity> velocitySupplier;

  private Supplier<AngularAcceleration> accelerationSupplier;

  private Supplier<Voltage> motorVoltageSupplier;

  private Supplier<Voltage> supplyVoltageSupplier;

  private Supplier<Current> statorCurrentSupplier;

  private Supplier<Current> supplyCurrentSupplier;

  public DiscreteMotorOutputSim() {
    outputVoltage = Volts.mutable(0);

    positionSupplier = () -> Rotations.of(0);
    velocitySupplier = () -> RotationsPerSecond.of(0);
    accelerationSupplier = () -> RotationsPerSecondPerSecond.of(0);
    motorVoltageSupplier = () -> outputVoltage;
    supplyVoltageSupplier = () -> Volts.of(0);
    statorCurrentSupplier = () -> Amps.of(0);
    supplyCurrentSupplier = () -> Amps.of(0);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    outputVoltage.mut_replace(voltage);
  }

  @Override
  public void setPosition(Angle newPosition) { }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    values.position.mut_replace(positionSupplier.get());
    values.velocity.mut_replace(velocitySupplier.get());
    values.acceleration.mut_replace(accelerationSupplier.get());
    values.motorVoltage.mut_replace(motorVoltageSupplier.get());
    values.supplyVoltage.mut_replace(supplyVoltageSupplier.get());
    values.statorCurrent.mut_replace(statorCurrentSupplier.get());
    values.supplyCurrent.mut_replace(supplyCurrentSupplier.get());
  }

  @Override
  public boolean configure() {
    return true;
  }

  public void usePosition(Supplier<Angle> positionSupplier) {
    this.positionSupplier = positionSupplier;
  }

  public void useVelocity(Supplier<AngularVelocity> velocitySupplier) {
    this.velocitySupplier = velocitySupplier;
  }

  public void useAcceleration(Supplier<AngularAcceleration> accelerationSupplier) {
    this.accelerationSupplier = accelerationSupplier;
  }

  public void useMotorVoltage(Supplier<Voltage> motorVoltageSupplier) {
    this.motorVoltageSupplier = motorVoltageSupplier;
  }

  public void useSupplyVoltage(Supplier<Voltage> supplyVoltageSupplier) {
    this.supplyVoltageSupplier = supplyVoltageSupplier;
  }

  public void useStatorCurrent(Supplier<Current> statorCurrentSupplier) {
    this.statorCurrentSupplier = statorCurrentSupplier;
  }

  public void useSupplyCurrent(Supplier<Current> supplyCurrentSupplier) {
    this.supplyCurrentSupplier = supplyCurrentSupplier;
  }
}
