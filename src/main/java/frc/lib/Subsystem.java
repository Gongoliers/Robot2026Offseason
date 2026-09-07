package frc.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem base class with some extra required functions */
public abstract class Subsystem extends SubsystemBase {

  /** Constructor that registers the subsystem to telemetry */
  public Subsystem(){
    Telemetry.registerSubsystem(this);
  }

  /** Initializes a subsystem's Shuffleboard tab */
  public abstract void initializeTab();
}
