package frc.lib.motors.motoroutput;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.CAN;
import frc.lib.configs.AbsoluteEncoderConfig;
import frc.lib.configs.MotorConfig;
import frc.lib.motors.MotorValues;

/** Motor output implementation for a single TalonFX controlled motor using a CANcoder for position and velocity measurements */
public class MotorOutputTalonFXCANcoder implements MotorOutput {
  
  /** Motor config */
  private final MotorConfig motorConfig;

  /** Absolute encoder config */
  private final AbsoluteEncoderConfig encoderConfig;

  /** TalonFX hardware */
  private final TalonFX motor;

  /** CANcoder hardware */
  private final CANcoder encoder;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  // Test
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;

  /** Voltage control request object */
  private VoltageOut voltage = new VoltageOut(0.0);

  /**
   * Motor output constructor
   *
   * @param config motor config used to configure motor
   * @param motorCAN CAN id and bus for TalonFX motor
   */
  public MotorOutputTalonFXCANcoder(MotorConfig motorConfig, AbsoluteEncoderConfig encoderConfig, CAN motorCAN, CAN encoderCAN) {

    // set config
    this.motorConfig = motorConfig;
    this.encoderConfig = encoderConfig;

    // create hardware and status signals
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());
    encoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    position = encoder.getPosition();
    velocity = encoder.getVelocity();
    acceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    supplyVoltage = motor.getSupplyVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();

    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        500,
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent,
        motorPosition,
        motorVelocity);
        
    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setControl(this.voltage.withOutput(voltage));
  }

  @Override
  public void setPosition(Angle newPosition) {
    encoder.setPosition(newPosition.times(encoderConfig.sensorToMechRatio()));
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
        supplyCurrent,
        motorPosition,
        motorVelocity);

    values.position.mut_replace(position.getValueAsDouble() / encoderConfig.sensorToMechRatio(), Rotations);
    values.velocity.mut_replace(velocity.getValueAsDouble() / encoderConfig.sensorToMechRatio(), RotationsPerSecond);
    values.acceleration.mut_replace(acceleration.getValueAsDouble(), RotationsPerSecondPerSecond);
    values.motorVoltage.mut_replace(motorVoltage.getValueAsDouble(), Volts);
    values.supplyVoltage.mut_replace(supplyVoltage.getValueAsDouble(), Volts);
    values.statorCurrent.mut_replace(statorCurrent.getValueAsDouble(), Amps);
    values.supplyCurrent.mut_replace(supplyCurrent.getValueAsDouble(), Amps);

    SmartDashboard.putNumber("Motor position", motorPosition.getValue().in(Rotations));
    SmartDashboard.putNumber("Motor velocity", motorVelocity.getValue().in(RotationsPerSecond));
  }

  @Override
  public boolean configure() {
    // TODO Allow failed configurations to return false
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();

    TalonFXConfiguration motorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(motorConfig.statorCurrentLimit())
                    .withSupplyCurrentLimit(motorConfig.supplyCurrentLimit()))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        motorConfig.ccwPositive()
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(
                        motorConfig.neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(motorConfig.rotorToSensorRatio()*motorConfig.sensorToMechRatio()));

    motorConfigurator.apply(motorConfiguration);

    CANcoderConfigurator encoderConfigurator = encoder.getConfigurator();

    CANcoderConfiguration encoderConfiguration =
      new CANcoderConfiguration()
        .withMagnetSensor(
          new MagnetSensorConfigs()
            .withMagnetOffset(encoderConfig.offset())
            .withSensorDirection(
              encoderConfig.ccwPositive()
                ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive));

    encoderConfigurator.apply(encoderConfiguration);

    return true;
  }
}
