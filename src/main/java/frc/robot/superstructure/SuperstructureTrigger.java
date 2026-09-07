package frc.robot.superstructure;

import java.util.function.Supplier;

/** Object used to allow extra state specific control */
public class SuperstructureTrigger {

  private final Supplier<Boolean> inputSupplier;

  private boolean pressed = false;
  
  /**
   * Constructs a superstructure trigger, used to pass input to superstructure for state specific manual control
   * 
   * @param inputSupplier supplies a simple true/false input for something ilke a button
   */
  public SuperstructureTrigger(Supplier<Boolean> inputSupplier) {
    this.inputSupplier = inputSupplier;
  }

  public boolean wasPressed() {
    if (inputSupplier.get()) {
      if (!pressed) {
        pressed = true;
        return true;
      }

      return false;
    } else {
      pressed = false;
      return false;
    }
  }

  public boolean held() {
    return inputSupplier.get();
  }
}
