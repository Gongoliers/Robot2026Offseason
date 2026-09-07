package frc.robot.superstructure;

public enum SuperstructureState {
  INIT(false),
  STOW(false),
  IDLE(true),
  FACE_HUB(true),
  INTAKE(true),
  SCORE(true),
  FEED(true),
  PASS(true),
  PASS_FAR(true),
  PASS_SOTM(true),
  SCORE_FROM_POSE(true),
  ALLOW_EXTERNAL_TURRET_CONTROL(true),
  ALLOW_EXTERNAL_TURRET_CONTROL_FACING_HUB(true),
  UNSAFE(false);

  public final boolean azimuthSafe;

  SuperstructureState(boolean azimuthSafe) {
    this.azimuthSafe = azimuthSafe;
  }
}
