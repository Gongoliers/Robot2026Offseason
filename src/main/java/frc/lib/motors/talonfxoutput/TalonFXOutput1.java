package frc.lib.motors.talonfxoutput;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.MotorValues;

/** TalonFX output implementation for a single motor */
public class TalonFXOutput1 implements TalonFXOutput {
  
  /** Config */
  private final MechanismConfig config;

  /** TalonFX hardware */
  private final TalonFX motor;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  /**
   * TalonFX output constructor
   * 
   * @param config config used to configure motor
   * @param motorCAN CAN id and bus for TalonFX motor
   */
  public TalonFXOutput1(MechanismConfig config, CAN motorCAN) {
    
    // set config
    this.config = config;

    // create hardware and status signals
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    supplyVoltage = motor.getSupplyVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent);
        
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void setControl(ControlRequest request) {
    motor.setControl(request);
  }

  @Override
  public void setPosition(Angle newPosition) {
    motor.setPosition(newPosition);
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent);

    values.position.mut_replace(position.getValueAsDouble(), Rotations);
    values.velocity.mut_replace(velocity.getValueAsDouble(), RotationsPerSecond);
    values.acceleration.mut_replace(acceleration.getValueAsDouble(), RotationsPerSecondPerSecond);
    values.motorVoltage.mut_replace(motorVoltage.getValueAsDouble(), Volts);
    values.supplyVoltage.mut_replace(supplyVoltage.getValueAsDouble(), Volts);
    values.statorCurrent.mut_replace(statorCurrent.getValueAsDouble(), Amps);
    values.supplyCurrent.mut_replace(supplyCurrent.getValueAsDouble(), Amps);
  }

  @Override
  public boolean configure() {
    // TODO Allow failed configurations to return false
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();

    TalonFXConfiguration motorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(config.motorConfig().statorCurrentLimit())
                    .withSupplyCurrentLimit(config.motorConfig().supplyCurrentLimit()))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        config.motorConfig().ccwPositive()
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(
                        config.motorConfig().neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(config.motorConfig().rotorToSensorRatio()*config.motorConfig().sensorToMechRatio()))
            .withSlot0(
                new Slot0Configs()
                    .withKP(config.feedbackControllerConfig().kP())
                    .withKI(config.feedbackControllerConfig().kI())
                    .withKD(config.feedbackControllerConfig().kD())
                    .withKV(config.feedforwardControllerConfig().kV())
                    .withKA(config.feedforwardControllerConfig().kA())
                    .withKS(config.feedforwardControllerConfig().kS())
                    .withKG(config.feedforwardControllerConfig().kG())
                    .withGravityType((config.feedforwardControllerConfig().armGravity()
                        ? GravityTypeValue.Arm_Cosine
                        : GravityTypeValue.Elevator_Static)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(config.motorConfig().voltageLimit())
                    .withPeakReverseVoltage(-config.motorConfig().voltageLimit()));

    motorConfigurator.apply(motorConfiguration);

    return true;
  }
}
