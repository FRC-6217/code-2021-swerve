/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.ButtonDebouncer;
import frc.robot.subsystems.DriveTrain;

public class JoyDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final Joystick joy;

  // Create Button Debouncer objects
  private ButtonDebouncer reverseHeading;
  private ButtonDebouncer invertFieldRelative;

  // Create class variables to hold robot control parameters
  private boolean isReversed = false; // Reverses robot direction when set to true
  private boolean fieldRelative = false; // Sets robot to field relative drive when set to true
  
  /**
   * Create a new JoyDrive
   * 
   * @param driveTrain The DriveTrain subsystem object of the scheduler
   * @param joy The drive joystick object
   */
  public JoyDrive(DriveTrain driveTrain, Joystick joy) {
    // Add the driveTrain subsystem object to the requirements of the JoyDrive Command
    addRequirements(driveTrain);

    // Pass parameters to class variables
    this.driveTrain = driveTrain;
    this.joy = joy;

    // Initialize button debouncer objects
    reverseHeading = new ButtonDebouncer(joy, 5);
    invertFieldRelative = new ButtonDebouncer(joy, 6);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    // Fetch the speed governor from the drive joystick -- Adjust axis value from 1 to 0 into 0 to 1
    double governor = Math.abs(1 - joy.getRawAxis(3));

    // Creates a deadzone of 30% and sets x, y, and z values to the governed joystick axes
    double x = (Math.abs(joy.getRawAxis(1)) > .3 ? joy.getRawAxis(1) * governor : 0.0);
    double y = (Math.abs(joy.getRawAxis(0)) > .3 ? joy.getRawAxis(0) * governor : 0.0);
    double z = (Math.abs(joy.getRawAxis(2)) > .3 ? joy.getRawAxis(2) * governor : 0.0);  
    

    // Control heading reversal with debounced joystick button
    if(reverseHeading.get()){
        isReversed = !isReversed;
    }

    // Control field relative toggling with debounced joystick button
    if(invertFieldRelative.get()){
      fieldRelative = !fieldRelative;
    }

    // Drive with joystick inputs and specific drive parameters
    if(isReversed){
      driveTrain.drive(-x, -y, z, fieldRelative);
    }
    else{
      driveTrain.drive(x, y, z, fieldRelative);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetain
    driveTrain.drive(0, 0, 0, false);
  }

  /**
   * Returns true when the command should end.
   * 
   * @return True when the command should end
   */
  @Override
  public boolean isFinished() {
    return false;
  }
  
  /**
   * Returns the string value of the JoyDrive Class
   * 
   * @return The string value of the JoyDrive Class
   */
  public String toString(){
    return "JoyDrive";
  }
}
