/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATE;
import frc.robot.subsystems.NotShooterIntake;

public class NotShooterIntakeCommand extends CommandBase {
  private NotShooterIntake notIntake;
  private STATE state;

  public NotShooterIntakeCommand(NotShooterIntake notIntake, STATE state) {
    addRequirements(notIntake);
    this.notIntake = notIntake;
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case FORWARDS:
        notIntake.suck();;
        break;
      case REVERSE:
        notIntake.spit();
        break;
      case OFF:
        notIntake.off();
        break;
      case UP:
        notIntake.off();
        break;
      case DOWN:
        notIntake.off();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    notIntake.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
