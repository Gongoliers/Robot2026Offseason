package frc.robot;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class TuningConstants {

    /**
     * Set to true to enable use of swerve acceleration and velocity limits.
     * If false, previous behavior will be used.
     */
    public static final boolean ENABLE_SWERVE_LIMITS = false;

    /**
     * Set to true to enable pass-on-the-move (POTM) changes.
     * If false, previous behavior will be used.
     */
    public static final boolean ENABLE_POTM_CHANGES = false;

    /**
     * Set to true to enable agitation changes.
     * If false, previous behavior will be used.
     */
    public static final boolean ENABLE_AGITATION_CHANGES = false;

    /**
     * The maximum translation velocity.
     * CTRE's swerve project generator reports that our theoretical top speed is 5.85 m/s.
     */
    public static final LinearVelocity SWERVE_VELOCITY_LIMIT = MetersPerSecond.of(2.75);

    /**
     * The maximum angular velocity.
     */
    public static final AngularVelocity SWERVE_ANGULAR_VELOCITY_LIMIT = RotationsPerSecond.of(0.75);

    /**
     * The maximum translation acceleration.
     * To start tuning this value, take the maximum drive speed and divide it by the desired time to reach maximum speed.
     * For a top speed of 5.85 m/s (our theoretical top speed) and a desired acceleration in 0.1 seconds, this
     * acceleration should be set to 58.5 m/s².
     * In previous years I have found that tuning the maximum acceleration isn't too helpful, because there is a fine
     * line between negatively impacting driver performance ("driving on ice") and being current-limited.
     */
    public static final LinearAcceleration SWERVE_LINEAR_ACCEL_LIMIT = MetersPerSecondPerSecond.of(6.0);

    /**
     * The maximum rotation acceleration.
     */
    public static final AngularAcceleration SWERVE_ANGULAR_ACCEL_LIMIT = RotationsPerSecondPerSecond.of(6.0);

    /**
     * The angle of the hood when passing in the POTM state.
     * See the {@link frc.robot.turret.TurretTargeter} class for the map from distance to angle.
     * Note that you can use Degrees.of(<some-number>) if you want the setpoint in degrees instead of rotations.
     * The conversion is performed automatically.
     */
    public static final Angle POTM_HOOD_SETPOINT = Rotations.of(0.07);

    /**
     * The velocity of the flywheel when passing in the POTM state.
     * See the {@link frc.robot.turret.TurretTargeter} class for the map from distance to velocity.
     */
    public static final AngularVelocity POTM_FLYWHEEL_SETPOINT = RotationsPerSecond.of(50);

    /**
     * Fudge factor for the fuel's exit velocity when passing in the POTM state.
     * The higher the fuel's exit velocity, the less compensation will be needed, because the fuel will have
     * less time to drift before it reaches its intended target.
     * This is a first-order approximation of the fuel's time of flight, which is what precise SOTM algorithm uses.
     */
    public static final LinearVelocity POTM_FUEL_EXIT_VELOCITY_FUDGE = MetersPerSecond.of(4);

    /**
     * Fudge factor for the calculated angle offset when passing in the POTM state.
     * If the POTM shot is lagging behind the intended target, increase this scalar.
     * If the POTM shot is leading the intended target, decrease this scalar.
     * That guide might be backwards, use your intution / try a few values to determine the right order.0
     */
    public static final double POTM_ANGLE_OFFSET_SCALAR_FUDGE = 1.0;

    /**
     * How far the intake agitation comes up. NOTE: CANNOT HAVE AN AGITATION OVER .1 BECAUSE FUEL WILL DO BAD THINGS
     */
    public static final Angle AGITATION_AMPLITUDE = Rotations.of(0.1);

    /**
     * How many times the intake agitates per second.
     */
    public static final double AGITATION_FREQUENCY = 2;

}
