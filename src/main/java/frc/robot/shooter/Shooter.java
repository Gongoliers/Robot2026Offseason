package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import frc.lib.MultithreadedSubsystem;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.MotorValues;
import frc.lib.motors.motoroutput.DiscreteMotorOutputSim;
import frc.lib.motors.motoroutput.MotorOutput;
import frc.robot.RobotConstants;

/** Shooter subsystem (handles velocity control of shooter rollers */
public class Shooter extends MultithreadedSubsystem {
  
  /** Shooter subsystem singleton */
  private static Shooter instance = null;

  /** Shooter motor output (specific type of motor output is defined in ShooterFactory) */
  private final MotorOutput motorOutput;

  /** Shooter motor output values (used to keep track of things like voltage, velocity, acceleration, stator current, temperature, etc) */
  private MotorValues motorValues = new MotorValues();

  /** Target angular velocity */
  private final MutAngularVelocity setpointVelocity;

  /** True if a manual voltage is set by runAtVoltage command */
  private boolean voltageSet;

  /** Output voltage */
  private final MutVoltage voltageOut;

  /** Feedback controller */
  private PIDController feedback;

  /** Feedforward controller */
  private SimpleMotorFeedforward feedforward;

  /** Shooter mechanism configuration that provides default values for motor control configuration and motor configuration */
  private MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(0.12266)
          .kA(0.015016)
          .kS(0.28614)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.12723)
          .kI(0.0)
          .kD(0.0)
          .build())
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(1)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  /**
   * Gets shooter subsystem instance
   * 
   * @return shooter subsystem instance
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  /** Shooter subsystem constructor */
  private Shooter() {
    motorOutput = ShooterFactory.createShooterMotor(config);
    motorOutput.configure();

    setpointVelocity = RotationsPerSecond.mutable(0);
    voltageSet = false;
    voltageOut = Volts.mutable(0);

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    if (motorOutput instanceof DiscreteMotorOutputSim discreteSim) {
      discreteSim.useVelocity(() -> setpointVelocity);
    }
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    ShuffleboardLayout stateTab = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateTab.addDouble("Motor voltage (V)", () -> motorValues.motorVoltage.in(Volts));
    //stateTab.addDouble("Supply voltage (V)", () -> motorValues.supplyVoltage.in(Volts));
    stateTab.addDouble("Stator current (A)", () -> motorValues.statorCurrent.in(Amps));
    stateTab.addDouble("Supply current (A)", () -> motorValues.supplyCurrent.in(Amps));
    //stateTab.addDouble("Position (rot)", () -> motorValues.position.in(Rotations));
    stateTab.addDouble("Velocity (rotps)", () -> motorValues.velocity.in(RotationsPerSecond));
    //stateTab.addDouble("Acceleration (rotpsps)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond));

    tab.addDouble("Setpoint (rotps)", () -> setpointVelocity.in(RotationsPerSecond));
  }

  @Override
  public void periodic() {}

  @Override
  public void fastPeriodic() {
    motorOutput.updateValues(motorValues, RobotConstants.FAST_PERIODIC_DURATION);

    double setpointVelRotationsPerSec = setpointVelocity.in(RotationsPerSecond);
    double velRotationsPerSec = motorValues.velocity.in(RotationsPerSecond);

    if (!voltageSet) {
      double feedbackVolts = 0.0;
      if (Math.abs(setpointVelRotationsPerSec) > 1 && !MathUtil.isNear(setpointVelRotationsPerSec, velRotationsPerSec, feedback.getErrorTolerance())) {
        feedbackVolts = feedback.calculate(velRotationsPerSec, setpointVelRotationsPerSec);
      }
      double feedforwardVolts = feedforward.calculate(setpointVelRotationsPerSec);

      SmartDashboard.putNumber("ff volts", feedforwardVolts);
      SmartDashboard.putNumber("fb volts", feedbackVolts);
      SmartDashboard.putNumber("total volts", feedforwardVolts + feedbackVolts);

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);
  }

  /**
   * Returns a command that allows for temporary manual voltage control of the shooter
   * 
   * @param voltageSupplier supplies a manual motor voltage to run at while the command is running
   * @return a command that allows for temporary manual voltage control of the shooter
   */
  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return this.run(() -> {
      voltageSet = true;
      voltageOut.mut_replace(voltageSupplier.get());
    }).finallyDo(() -> voltageSet = false);
  }

  /**
   * Returns true if shooter is near its setpoint
   * 
   * @param tolerance how close it has to be to its setpoint to be considered near that setpoint
   * @return true if shooter is near its setpoint
   */
  public boolean nearSetpoint(AngularVelocity tolerance) {
    return motorValues.velocity.isNear(setpointVelocity, tolerance);
  }

  public void setSetpoint(AngularVelocity velocitySetpoint) {
    setpointVelocity.mut_replace(velocitySetpoint);
  }

  public AngularVelocity getSetpoint() {
    return setpointVelocity;
  }

  public MechanismConfig getConfig() {
    return config;
  }

  public MotorValues getValues() {
    return motorValues;
  }
}
