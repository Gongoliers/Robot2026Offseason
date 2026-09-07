package frc.robot.kicker;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.MotorValues;
import frc.lib.motors.motoroutput.DiscreteMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.robot.RobotConstants;

public class Kicker extends Subsystem {
  
  private static Kicker instance = null;

  private final MotorOutput motorOutput;

  private MotorValues motorValues = new MotorValues();

  private KickerState state;

  private boolean voltageSet;

  private final MutVoltage voltageOut;

  private final PIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private final MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(0.67867)
          .kA(0.013461)
          .kS(0.1338)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.8)
          .kI(0)
          .kD(0)
          .build())
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(true)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(3*(36/15))
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  public static Kicker getInstance() {
    if (instance == null) {
      instance = new Kicker();
    }

    return instance;
  }

  private Kicker() {
    motorOutput = KickerFactory.createKickerMotor(config);
    motorOutput.configure();

    state = KickerState.STOP;

    voltageOut = Volts.mutable(0);

    voltageSet = false;

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    if (motorOutput instanceof DiscreteMotorOutputSim discreteSim) {
      discreteSim.useVelocity(() -> state.velocity);
    }
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Kicker");

    ShuffleboardLayout stateTab = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateTab.addDouble("Motor voltage (V)", () -> motorValues.motorVoltage.in(Volts));
    //stateTab.addDouble("Supply voltage (V)", () -> motorValues.supplyVoltage.in(Volts));
    stateTab.addDouble("Stator current (A)", () -> motorValues.statorCurrent.in(Amps));
    //stateTab.addDouble("Supply current (A)", () -> motorValues.supplyCurrent.in(Amps));
    //stateTab.addDouble("Position (rot)", () -> motorValues.position.in(Rotations));
    stateTab.addDouble("Velocity (rotps)", () -> motorValues.velocity.in(RotationsPerSecond));
    //stateTab.addDouble("Acceleration (rotpsps)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond));

    tab.addString("Current state", () -> state.name());
    tab.addDouble("Setpoint (rotps)", () -> state.velocity.in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    motorOutput.updateValues(motorValues, RobotConstants.PERIODIC_DURATION);

    double setpointRotationsPerSecond = state.velocity.in(RotationsPerSecond);
    double velocityRotationsPerSecond = motorValues.velocity.in(RotationsPerSecond);

    if (!voltageSet) {
      double feedbackVolts = MathUtil.clamp(feedback.calculate(velocityRotationsPerSecond, setpointRotationsPerSecond), -6, 6);
      double feedforwardVolts = feedforward.calculate(setpointRotationsPerSecond);

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);
  }

  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return this.run(() -> {
      voltageSet = true;
      Voltage voltageRequest = voltageSupplier.get();
      voltageOut.mut_replace(voltageRequest);
    }).finallyDo(() -> voltageSet = false);
  }

  /** 
   * Instantaneously changes the kicker's control state if the kicker subsystem is not currently required by a command
   * 
   * @param kickerState New kicker state
   */
  public void setState(KickerState kickerState) {
    if (this.getCurrentCommand() == null) {
      state = kickerState;
    }
  }

  /**
   * Returns a command that changes the kicker's control state and waits until at that state
   * 
   * @param kickerState New kicker state
   * @return A command that changes the kicker's control state and waits until at that state
   */
  public Command goToState(KickerState kickerState) {
    return Commands.sequence(
      this.runOnce(() -> state = kickerState),
      Commands.waitUntil(this::atTargetState)
    );
  }

  public boolean atTargetState() {
    return motorValues.velocity.isNear(state.velocity, RotationsPerSecond.of(1));
  }

  public KickerState getState() {
    return state;
  }

  public MechanismConfig getConfig() {
    return config;
  }

  public MotorValues getValues() {
    return motorValues;
  }
}
