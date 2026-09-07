package frc.lib;

import java.util.ArrayList;
import java.util.List;

/** Handles telemetry with Shuffleboard */
public class Telemetry {

  /** List of subsytems to be utilized by initializeTabs */
  private static List<Subsystem> subsystemTabs = new ArrayList<Subsystem>();

  /** 
   * registers a Subsytem to have their initializeTab function called
   * 
   * @param subsystem Subsystem class to register
   */
  public static void registerSubsystem(Subsystem subsystem){
    subsystemTabs.add(subsystem);
  }

  /** runs the initializeTabe function for each of the subsytems that have been registered */
  public static void initializeTabs() {
    for (Subsystem subsystem : subsystemTabs) {
      subsystem.initializeTab();
    }
  }
}
