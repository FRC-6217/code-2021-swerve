package frc.robot.libraries;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/** ButtonDebouncer */
public class ButtonDebouncer {
    Joystick joystick;
    int buttonnum;
    double latest;
    double debounce_period;

    /**
     * Creates a new ButtonDebouncer
     * 
     * @param joystick The Joystick object to recieve button input from
     * @param buttonnum The button ID of the specific joystick button
     */
    public ButtonDebouncer(Joystick joystick, int buttonnum){
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = .5;
    }

    /**
     * Creates a new ButtonDebouncer
     * 
     * @param joystick The Joystick object to recieve button input from
     * @param buttonnum The button ID of the specific joystick button
     * @param period The desired period of debounce
     */
    public ButtonDebouncer(Joystick joystick, int buttonnum, float period){
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = period;
    }

    /**
     * Set the debounce period of the button
     * 
     * @param period The desired debounce period of the button
     */
    public void setDebouncePeriod(float period){
        this.debounce_period = period;
    }

    /**
     * Returns the state of the button with debounce protection
     * 
     * @return The state of the button with debounce protection
     */
    public boolean get(){
        double now = Timer.getFPGATimestamp();
        if(joystick.getRawButton(buttonnum)){
            if((now-latest) > debounce_period){
                latest = now;
                return true;
            }
        }
        return false;
    }
}
