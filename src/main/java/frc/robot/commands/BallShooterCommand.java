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

public class BallShooterCommand extends CommandBase {
  boolean ifOn;
  BallShooter shooter;

  private double top;
  private double bottom;
  /**
   * Creates a new ShooterCommand.
   */
  public BallShooterCommand(BallShooter shooter, boolean ifOn, double top, double bottom) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.ifOn = ifOn;

    this.top = top;
    this.bottom = bottom;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  SmartDashboard.putNumber("Top Shoot Set", 2700);
  //  SmartDashboard.putNumber("Bottom Shoot Set", 2700);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO make distance based
    top = SmartDashboard.getNumber("Top Shoot Set", 0);
    bottom = SmartDashboard.getNumber("Bottom Shoot Set", 0);
    shooter.on(top, bottom);
    // shooter.on();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!ifOn) {
    //   return true;
    // }
    return false;
  }
}
