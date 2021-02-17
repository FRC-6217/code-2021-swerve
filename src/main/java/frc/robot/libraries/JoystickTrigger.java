package frc.robot.libraries;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class JoystickTrigger extends Button {
  private final GenericHID joystick;
  private final int triggerAxisNumber;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickTrigger(GenericHID joystick, int triggerAxisNumber) {
    requireNonNullParam(joystick, "joystick", "JoystickTrigger");

    this.joystick = joystick;
    this.triggerAxisNumber = triggerAxisNumber;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    if(joystick.getRawAxis(triggerAxisNumber) > 0.5){
      return true;
    }
    else{
      return false;
    }
  }
}
