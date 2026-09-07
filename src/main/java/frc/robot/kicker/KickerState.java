package frc.robot.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public enum KickerState {
  STOP(RotationsPerSecond.of(0)),
  RUN(RotationsPerSecond.of(14));

  public final AngularVelocity velocity;

  KickerState(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
