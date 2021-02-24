/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARM_LIFT_CONSTANTS;
import frc.robot.Constants.ARM_LIFT_CONSTANTS.STATE;
import frc.robot.subsystems.ArmLift;

public class ArmLiftCommand extends CommandBase {
  private ArmLift arm;
  private ARM_LIFT_CONSTANTS.STATE state;
  private boolean finished;
  
  public ArmLiftCommand(ArmLift arm, ARM_LIFT_CONSTANTS.STATE state) {
    addRequirements(arm);
    this.arm = arm;
    this.state = state;
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == STATE.UP) {
      finished = arm.upMotorLimited();
    }

    else if(state == STATE.DOWN) {
      finished = arm.downMotorLimited();
    }

    else {
      finished = true; 
      arm.offMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.offMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
