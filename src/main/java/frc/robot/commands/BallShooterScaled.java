// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.Distance;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.LimeLight;

public class BallShooterScaled extends CommandBase {
  private BallShooter shooter;
  private Distance distance;

  /** Creates a new BallShooterScaled. */
  public BallShooterScaled(BallShooter shooter, Distance distance) {
    addRequirements(shooter);

    this.shooter = shooter;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
