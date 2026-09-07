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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.MotorValues;

/** TalonFX output implementation for two motors */
public class TalonFXOutput2 implements TalonFXOutput {
  
  /** Config */
  private final MechanismConfig config;

  /** TalonFX hardware */
  private final TalonFX leader;
  private final TalonFX follower;

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
   * @param leaderCAN CAN id and bus for leader motor
   * @param followerCAN CAN id and bus for follower motor
   * @param motorAlignment Motor alignment value (inverts follower)
   */
  public TalonFXOutput2(MechanismConfig config, CAN leaderCAN, CAN followerCAN, MotorAlignmentValue motorAlignment) {
    
    // set config
    this.config = config;

    // create hardware and status signals
    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    follower = new TalonFX(followerCAN.id(), followerCAN.bus());

    follower.setControl(new Follower(leaderCAN.id(), motorAlignment));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    acceleration = leader.getAcceleration();
    motorVoltage = leader.getMotorVoltage();
    supplyVoltage = leader.getSupplyVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent);
        
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);
  }

  @Override
  public void setControl(ControlRequest request) {
    leader.setControl(request);
  }

  @Override
  public void setPosition(Angle newPosition) {
    leader.setPosition(newPosition);
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
    TalonFXConfigurator leaderConfigurator = leader.getConfigurator();
    TalonFXConfigurator followerConfigurator = follower.getConfigurator();

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

    leaderConfigurator.apply(motorConfiguration);
    followerConfigurator.apply(motorConfiguration);

    return true;
  }
}
