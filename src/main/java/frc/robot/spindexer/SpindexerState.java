package frc.robot.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public enum SpindexerState {
  STOP(RotationsPerSecond.of(0)),
  RUN(RotationsPerSecond.of(10));

  public final AngularVelocity velocity;

  SpindexerState(AngularVelocity velocity) {
    this.velocity = velocity;
  }
}
