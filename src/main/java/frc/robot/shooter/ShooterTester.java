package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motors.MotorValues;

/** Class with methods to run sysid and test the shooter */
public class ShooterTester {
  
  /** Shooter tester instance */
  private static ShooterTester instance = null;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /** Shooter motor values */
  private MotorValues motorValues;

  /** List of angular velocities measured during testing */
  private ArrayList<Double> velocityAccumulator;

  /** Keeps track of max distance of velocity from setpoint velocity during a disturbance */
  private double maxVelocityError;

  /** Keeps track of the max velocity errors during each disturbance */
  private ArrayList<Double> maxVelocityErrors;

  /** How close velocity should be to target velocity to be considered at that velocity */
  private AngularVelocity velocityTolerance;

  /** Times to reach velocity threshold */
  private ArrayList<Time> returnTimes;

  /** Keeps track of how many frames the shooter has been within velocity tolerance */
  private int framesWithinTolerance;

  /** Keeps track of how many frames since shooter disturbance */
  private int framesSinceDisturbance;

  /**
   * Gets shooter tester instance
   * 
   * @return shooter tester instance
   */
  public static ShooterTester getInstance() {
    if (instance == null) {
      instance = new ShooterTester();
    }

    return instance;
  }

  private ShooterTester() {
    shooter = Shooter.getInstance();

    motorValues = new MotorValues();
    velocityAccumulator = new ArrayList<Double>();
    maxVelocityError = 0;
    maxVelocityErrors = new ArrayList<Double>();
    velocityTolerance = RotationsPerSecond.of(1);
    returnTimes = new ArrayList<Time>();
    framesWithinTolerance = 0;
    framesSinceDisturbance = 0;
  }

  public Command findVelocityVariance(AngularVelocity testVelocity) {
    return Commands.runOnce(() -> {
      velocityAccumulator = new ArrayList<Double>();
      shooter.setSetpoint(testVelocity);
    })
    .andThen(Commands.waitSeconds(1)) // Let the motor spin up
    .andThen(Commands.race(
      Commands.waitSeconds(5),
      Commands.run(() -> {
        motorValues = shooter.getValues();
        velocityAccumulator.add(motorValues.velocity.in(RotationsPerSecond));
      })
    ))
    .andThen(Commands.runOnce(() -> {
      shooter.setSetpoint(RotationsPerSecond.of(0.0));

      double maxVelRotationsPerSec = Double.NEGATIVE_INFINITY;
      double minVelRotationsPerSec = Double.POSITIVE_INFINITY;

      for (double velocityRotationsPerSec : velocityAccumulator) {
        maxVelRotationsPerSec = Math.max(velocityRotationsPerSec, maxVelRotationsPerSec);
        minVelRotationsPerSec = Math.min(velocityRotationsPerSec, minVelRotationsPerSec);
      }
    
      double center = (maxVelRotationsPerSec + minVelRotationsPerSec)/2;
      double maxError = maxVelRotationsPerSec - center;

      velocityTolerance = RotationsPerSecond.of(maxError*1.5);
    }));
  }

  public Command runTests(AngularVelocity testVelocity) {
    NumberFormat niceFormat = new DecimalFormat("#0.00");
    return Commands.run(() -> {
      shooter.setSetpoint(testVelocity);

      motorValues = shooter.getValues();

      boolean inTolerance = MathUtil.isNear(testVelocity.in(RotationsPerSecond), motorValues.velocity.in(RotationsPerSecond), velocityTolerance.in(RotationsPerSecond));

      SmartDashboard.putNumber("Upper bound", testVelocity.plus(velocityTolerance).in(RotationsPerSecond));
      SmartDashboard.putNumber("Lower bound", testVelocity.minus(velocityTolerance).in(RotationsPerSecond));
      SmartDashboard.putBoolean("Within tolerance", inTolerance);

      if (framesSinceDisturbance > 0) { // If frames since disturbance has started counting, continue
        framesSinceDisturbance += 1;
      }

      if (inTolerance) {
        framesWithinTolerance += 1;

        if (framesSinceDisturbance < 4) { // if it's a tiny fake disturbance ignore it
          framesSinceDisturbance = 0;
        }
      } else {
        framesWithinTolerance = 0;

        if (framesSinceDisturbance == 0) { // If frames since disturbance hasn't started counting, start counting it
          framesSinceDisturbance += 1;
        }
      }

      maxVelocityError = Math.max(maxVelocityError, testVelocity.minus(motorValues.velocity).abs(RotationsPerSecond));

      // If the shooter has been in tolerance for 0.4s
      if (framesWithinTolerance > 20) {
        if (framesSinceDisturbance > 3) { // this is needed i forgot why lol
          double secondsSinceDisturbance = (framesSinceDisturbance - 20) * 0.02;
          returnTimes.add(Seconds.of(secondsSinceDisturbance));
          System.out.println("Returned in "+niceFormat.format(secondsSinceDisturbance)+"s");

          maxVelocityErrors.add(maxVelocityError);
          System.out.println("Max velocity loss: "+niceFormat.format(maxVelocityError)+"rot/s ("+niceFormat.format(maxVelocityError/testVelocity.in(RotationsPerSecond)*100)+"%)");
          maxVelocityError = 0.0;
        }

        framesSinceDisturbance = 0;
      }
    }).finallyDo(() -> {
      shooter.setSetpoint(RotationsPerSecond.of(0));

      System.out.println("Spun up in "+returnTimes.get(0).in(Seconds)+"s");

      double timesAvg = 0.0;
      for (int i = 1; i < returnTimes.size(); i++) {
        timesAvg += returnTimes.get(i).in(Seconds);
      }
      timesAvg /= returnTimes.size()-1;

      double velocityErrorAvg = 0.0;
      for (int i = 1; i < maxVelocityErrors.size(); i++) {
        velocityErrorAvg += maxVelocityErrors.get(i);
      }
      velocityErrorAvg /= maxVelocityErrors.size()-1;

      System.out.println("Average return time: "+niceFormat.format(timesAvg)+"s");
      System.out.println("Average velocity loss: "+niceFormat.format(velocityErrorAvg)+"rot/s ("+niceFormat.format(velocityErrorAvg/testVelocity.in(RotationsPerSecond)*100)+"%)");
    });
  }
}