/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallShooter;

public class BallShooterCommandAuto extends CommandBase {
  boolean ifOn;
  BallShooter shooter;
  /**
   * Creates a new ShooterCommand.
   */
  public BallShooterCommandAuto(BallShooter shooter, boolean ifOn) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.ifOn = ifOn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    //TODO make distance based
    if(ifOn){
      shooter.on(2500, 2500);
    }
    else{
      shooter.off();
    }
    SmartDashboard.putBoolean("On?", ifOn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
