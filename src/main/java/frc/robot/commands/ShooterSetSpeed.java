// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.BallShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSetSpeed extends CommandBase {
  
  private int topRPM = 0;
  private int bottomRPM = 0;
  private BallShooter shooter;
  /** Creates a new shooterSetSpeed. */
    public ShooterSetSpeed(int topRPM, int bottomRPM) {
      // addRequirements(shooter);
      this.topRPM = topRPM;
      this.bottomRPM = bottomRPM;      
      // this.shooter = shooter;
      // eg. requires(chassis);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putNumber("Top Shoot Set", topRPM);
    SmartDashboard.putNumber("Bottom Shoot Set", bottomRPM);
    // shooter.setspeed(topRPM, bottomRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
