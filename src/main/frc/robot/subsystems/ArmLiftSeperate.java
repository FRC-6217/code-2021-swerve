/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATE;
import frc.robot.Constants.SIDE;


public class ArmLiftSeperate extends CommandBase {


  private ArmLift armLift;
  SIDE side;
  STATE state;

  /**
   * Creates a new ArmLiftSeperate.
   */
  public ArmLiftSeperate(ArmLift armLift, SIDE side, STATE state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armLift);
    this.armLift = armLift;
    this.side = side;
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // armLift.moveArm(side, state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // armLift.moveArm(side, STATE.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
