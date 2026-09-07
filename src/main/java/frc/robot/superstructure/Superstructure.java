package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerState;
import frc.robot.spindexer.Spindexer;
import frc.robot.spindexer.SpindexerState;
import frc.robot.turret.Turret;

/**
 * Superstructure subsystem
 * 
 * Manages automated control of all other subsystems
 */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Intake subsystem reference */
  private final Intake intake;

  /** Kicker subsystem reference */
  private final Kicker kicker;

  /** Spindexer subsystem reference */
  private final Spindexer spindexer;

  /** Turret subsystem reference */
  private final Turret turret;

  /** Superstructure state */
  private SuperstructureState state;

  // State specific control variables

  /** Superstructure trigger used to fire fuel when in a state that allows for manual firing */
  private SuperstructureTrigger manualFireTrigger;

  /** Superstructure trigger used to agitate fuel when in a state that allows for manual agitation */
  private SuperstructureTrigger manualAgitateTrigger;

  /**
   * Gets superstructure subsystem instance
   * 
   * @return superstructure subsystem instance
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  /** Superstructure subsystem constructor */
  private Superstructure() {
    intake = Intake.getInstance();
    kicker = Kicker.getInstance();
    spindexer = Spindexer.getInstance();
    turret = Turret.getInstance();

    state = SuperstructureState.INIT;
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("superstructure");

    tab.addString("state", () -> state.name());
    tab.addString("safe state", () -> getSafeState().name());
    tab.addBoolean("at target state", () -> atTargetState());
  }

  @Override
  public void periodic() {
    switch (state) {
      case SCORE:
        if (turret.atTargetState()) {
          kicker.setState(KickerState.RUN);
          spindexer.setState(SpindexerState.RUN);
        } else{
          kicker.setState(KickerState.STOP);
          spindexer.setState(SpindexerState.STOP);
        }
        break;
      case FEED:
        kicker.setState(KickerState.RUN);
        spindexer.setState(SpindexerState.RUN);
        break;
      case SCORE_FROM_POSE:
        if (manualFireTrigger.held()) {
          kicker.setState(KickerState.RUN);
          spindexer.setState(SpindexerState.RUN);
        } else {
          kicker.setState(KickerState.STOP);
          spindexer.setState(SpindexerState.STOP);
        }
        
        break;
      case PASS, PASS_FAR, PASS_SOTM:
        if (manualFireTrigger.held() && turret.atTargetState()) {
          kicker.setState(KickerState.RUN);
          spindexer.setState(SpindexerState.RUN);
        } else {
          kicker.setState(KickerState.STOP);
          spindexer.setState(SpindexerState.STOP);
        }

        if (manualAgitateTrigger.held()) {
          intake.setState(IntakeState.AGITATE);
        } else {
          intake.setState(IntakeState.INTAKE);
        }

        break;
      default:
        break;
    }
  }

  public boolean atTargetState() {
    return turret.atTargetState()
        && intake.atTargetState()
        && kicker.atTargetState()
        && spindexer.atTargetState();
  }

  public SuperstructureState getState() {
    return state;
  }

  public SuperstructureState getSafeState() {
    if (atTargetState()) {
      return state;
    }

    return SuperstructureState.UNSAFE;
  }

  public Command init() {
    return Commands.parallel(
      turret.stow(),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      intake.goToState(IntakeState.INIT)
    ).finallyDo(() -> state = SuperstructureState.INIT);
  }

  public Command stow() {
    return Commands.parallel(
      turret.stow(),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      intake.goToState(IntakeState.STOW)
    ).finallyDo(() -> state = SuperstructureState.STOW);
  }

  public Command idle() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      turret.stow()
    ).finallyDo(() -> state = SuperstructureState.IDLE);
  }

  public Command faceHub() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      turret.faceHub()
    ).finallyDo(() -> state = SuperstructureState.FACE_HUB);
  }

  public Command intake() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.INTAKE),
      turret.faceHub()
    )).finallyDo(() -> state = SuperstructureState.INTAKE);
  }

  public Command score() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.AGITATE),
      turret.targetHub()
    ))
    .finallyDo(() -> state = SuperstructureState.SCORE);
  }

  public Command scoreFromPose(Pose2d scorePose, SuperstructureTrigger shootTrigger) {
    return Commands.parallel(
      Commands.runOnce(() -> manualFireTrigger = shootTrigger),
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.AGITATE),
      turret.targetHubFromPose(scorePose)
    ))
    .finallyDo(() -> state = SuperstructureState.SCORE_FROM_POSE);
  }

  public Command feed() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.AGITATE),
      turret.allowExternalControlFacingHub()
    ))
    .finallyDo(() -> state = SuperstructureState.FEED);
  }

  public Command pass(SuperstructureTrigger agitateTrigger, SuperstructureTrigger shootTrigger) {
    return Commands.parallel(
      Commands.runOnce(() -> {
        manualAgitateTrigger = agitateTrigger;
        manualFireTrigger = shootTrigger;
      }),
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.INTAKE),
      turret.pass()
    ))
    .finallyDo(() -> state = SuperstructureState.PASS);
  }

  public Command passFar(SuperstructureTrigger agitateTrigger, SuperstructureTrigger shootTrigger) {
    return Commands.parallel(
      Commands.runOnce(() -> {
        manualAgitateTrigger = agitateTrigger;
        manualFireTrigger = shootTrigger;
      }),
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.INTAKE),
      turret.passFar()
    ))
    .finallyDo(() -> state = SuperstructureState.PASS_FAR);
  }

  public Command passSOTM(SuperstructureTrigger agitateTrigger, SuperstructureTrigger shootTrigger) {
    return Commands.parallel(
      Commands.runOnce(() -> {
        manualAgitateTrigger = agitateTrigger;
        manualFireTrigger = shootTrigger;
      }),
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.INTAKE),
      turret.passSOTM()
    ))
    .finallyDo(() -> state = SuperstructureState.PASS_SOTM);
  }
}
