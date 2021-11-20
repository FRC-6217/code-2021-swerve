// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class JoyDriveForward extends CommandBase {
  private final DriveTrain driveTrain;
  private final double speed;
  /** Creates a new JoyDriveForward. */
  public JoyDriveForward(DriveTrain driveTrain, double speed) {// Add the driveTrain subsystem object to the requirements of the JoyDrive Command
    addRequirements(driveTrain);

    // Pass parameters to class variables
    this.driveTrain = driveTrain;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    driveTrain.drive(speed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
