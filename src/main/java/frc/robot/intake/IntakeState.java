package frc.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public enum IntakeState {
  INIT(Rotations.of(0.275), RotationsPerSecond.of(0)),
  STOW(Rotations.of(0.25), RotationsPerSecond.of(0)),
  OUT(Rotations.of(-0.05), RotationsPerSecond.of(0)),
  INTAKE(Rotations.of(-0.05), RotationsPerSecond.of(90)),
  AGITATE(null, RotationsPerSecond.of(40));

  public final Angle pivotSetpoint;
  public final AngularVelocity rollerSetpoint;

  IntakeState(Angle pivotSetpoint, AngularVelocity rollerSetpoint) {
    this.pivotSetpoint = pivotSetpoint;
    this.rollerSetpoint = rollerSetpoint;
  }
}
