package frc.lib.configs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Feedforward config
 *
 * @param kS voltage to overcome static friction
 * @param kG voltage to overcome gravity
 * @param kV voltage to overcome friction or drag that reduces velocity
 * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
 * @param armGravity if true, use arm gravity, if falls, use elevator
 */
public record FeedforwardControllerConfig(double kS, double kG, double kV, double kA, boolean armGravity) {

  /**
   * Construct simple feedforward without kG
   *
   * @param kS voltage to overcome static friction
   * @param kV voltage to overcome friction or drag that reduces velocity
   * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
   */
  public FeedforwardControllerConfig(double kS, double kV, double kA) {
    this(kS, 0.0, kV, kA, false);
  }

  /**
   * Creates a simple motor feedforward using this config
   *
   * @return a simple motor feedforward using this config
   */
  public SimpleMotorFeedforward createSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Creates an arm feedforward using this config
   *
   * @return an arm feedforward using this config
   */
  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }

  /**
   * Creates an elevator feedforward using this config
   *
   * @return an elevator feedforward using this config
   */
  public ElevatorFeedforward createElevatorFeedforward() {
    return new ElevatorFeedforward(kS, kG, kV, kA);
  }

  /** Easier and more modular way to construct feedforward controller configs */
  public static class FeedforwardControllerBuilder {
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private boolean armGravity;

    private FeedforwardControllerBuilder(double kS, double kG, double kV, double kA, boolean armGravity) {
      this.kS = kS;
      this.kG = kG;
      this.kV = kV;
      this.kA = kA;
      this.armGravity = armGravity;
    }

    /**
     * Returns a builder with default values
     *
     * @return a builder with default values
     */
    public static FeedforwardControllerBuilder defaults() {
      return new FeedforwardControllerBuilder(0.0, 0.0, 0.0, 0.0, false);
    }

    /**
     * Returns a builder with values copied from the input config
     *
     * @param config config to be copied
     * @return a builder with values copied from the input config
     */
    public static FeedforwardControllerBuilder edit(FeedforwardControllerConfig config) {
      return new FeedforwardControllerBuilder(config.kS(), config.kG(), config.kV(), config.kA(), config.armGravity());
    }

    public FeedforwardControllerBuilder kS(double kS) {
      this.kS = kS;
      return this;
    }

    public FeedforwardControllerBuilder kG(double kG) {
      this.kG = kG;
      return this;
    }

    public FeedforwardControllerBuilder kV(double kV) {
      this.kV = kV;
      return this;
    }

    public FeedforwardControllerBuilder kA(double kA) {
      this.kA = kA;
      return this;
    }

    public FeedforwardControllerBuilder armGravity(boolean armGravity) {
      this.armGravity = armGravity;
      return this;
    }

    /**
     * Returns the builder as a config with private immutable values
     *
     * @return the builder as a config with private immutable values
     */
    public FeedforwardControllerConfig build() {
      return new FeedforwardControllerConfig(this.kS, this.kG, this.kV, this.kA, this.armGravity);
    }
  }
}
