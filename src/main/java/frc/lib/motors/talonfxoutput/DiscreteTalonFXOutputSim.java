package frc.lib.motors.talonfxoutput;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.*;
import frc.lib.motors.MotorValues;

public class DiscreteTalonFXOutputSim implements TalonFXOutput {
  
  private MutVoltage voltage;

  private MutAngle position;

  private Supplier<Angle> positionSupplier;

  private Supplier<AngularVelocity> velocitySupplier;

  private Supplier<AngularAcceleration> accelerationSupplier;

  private Supplier<Voltage> motorVoltageSupplier;

  private Supplier<Voltage> supplyVoltageSupplier;

  private Supplier<Current> statorCurrentSupplier;

  private Supplier<Current> supplyCurrentSupplier;

  public DiscreteTalonFXOutputSim() {
    voltage = Volts.mutable(0);
    position = Rotations.mutable(0);

    positionSupplier = () -> position;
    velocitySupplier = () -> RotationsPerSecond.of(0);
    accelerationSupplier = () -> RotationsPerSecondPerSecond.of(0);
    motorVoltageSupplier = () -> voltage;
    supplyVoltageSupplier = () -> Volts.of(0);
    statorCurrentSupplier = () -> Amps.of(0);
    supplyCurrentSupplier = () -> Amps.of(0);
  }

  @Override
  public void setControl(ControlRequest request) {
    if (request instanceof VoltageOut) {
      voltage.mut_replace(Double.parseDouble(request.getControlInfo().get("Output")), Volts);
    }
  }

  @Override
  public void setPosition(Angle newPosition) {
    position.mut_replace(newPosition);
  }

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
