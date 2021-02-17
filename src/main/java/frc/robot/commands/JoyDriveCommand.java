/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class JoyDriveCommand extends CommandBase {
  private final driveTrain driveTrain;

  private Joystick joy;
  private double x;
  private double y;
  private double z;
  private boolean gyroButtonForward;
  private boolean gyroButtonBackward;
  private double governer;
  
  private boolean isReversed;
  private double x1;
  private double y1;
  private int direction = 1;
  
  /**
   * Creates a new JoyDrive.
   */
  public JoyDriveCommand(driveTrain train, Joystick joy) {
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
    y = joy.getRawAxis(0);
    x = joy.getRawAxis(1);
    z = joy.getRawAxis(2);
    // gyroButtonForward = joy.getRawButton(5);
    // gyroButtonBackward = joy.getRawButton(6);        
    governer = joy.getRawAxis(3);

    // if(gyroButtonForward){
    //     driveTrain.resetGyro();
    //     isReversed = false;
    // }
    // else if(gyroButtonBackward){
    //     driveTrain.resetGyro();
    //     isReversed = true;
    // }
    
    x = (Math.abs(x) > .3 ? x : 0.0);
    y = (Math.abs(y) > .3 ? y : 0.0);
    z = (Math.abs(z) > .3 ? z : 0.0);

    x1 = driveTrain.TransformX(x, y, isReversed);
    y1 = driveTrain.TransformY(x, y, isReversed);

    if(isReversed){
      driveTrain.Drive (y, -x, -z, Math.abs(governer-1));
    }
    else{
      driveTrain.Drive (-y, x, -z, Math.abs(governer-1));
    }

    
    gyroButtonForward = false;
    gyroButtonBackward = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.Drive(0, 0, 0, 0);
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
