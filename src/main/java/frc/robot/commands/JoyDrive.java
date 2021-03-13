/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class JoyDrive extends CommandBase {
  private final DriveTrain driveTrain;

  private final Joystick joy;
  private double governer;
  private double x;
  private double y;
  private double z;
  private boolean gyroButtonForward;
  private boolean gyroButtonBackward;
  private boolean fieldRelative = false;
  
  private boolean isReversed;
  
  /**
   * Creates a new JoyDrive.
   */
  public JoyDrive(DriveTrain train, Joystick joy) {
    addRequirements(train);

    driveTrain = train;
    this.joy = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    governer = Math.abs(1 - joy.getRawAxis(3));

    y = (Math.abs(x) > .3 ? joy.getRawAxis(0) * governer : 0.0);
    x = (Math.abs(y) > .3 ? joy.getRawAxis(1) * governer : 0.0);
    z = (Math.abs(z) > .3 ? joy.getRawAxis(2) * governer : 0.0);
    
    gyroButtonForward = joy.getRawButton(5);
    gyroButtonBackward = joy.getRawButton(6);        

    if(gyroButtonForward){
        driveTrain.zeroHeading();
        isReversed = false;
    }
    else if(gyroButtonBackward){
        driveTrain.zeroHeading();
        isReversed = true;
    }

    if(joy.getRawButton(7) && !fieldRelative){
      fieldRelative = true;
    }
    else if(joy.getRawButton(7) && fieldRelative){
      fieldRelative = false;
    }

    if(isReversed){
      driveTrain.drive(-y, -y, z, fieldRelative);
    }
    else{
      driveTrain.drive(x, y, z, fieldRelative);
    }

    
    gyroButtonForward = false;
    gyroButtonBackward = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public String toString(){
    return "JoyDrive";
  }
}
