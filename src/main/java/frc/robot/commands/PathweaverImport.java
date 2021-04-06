// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVE_TRAIN_CONSTANTS;
import frc.robot.Constants.PATHFINDER_CONSTANTS;
import frc.robot.Constants.SWERVE_MODULE_CONSTANTS;
import frc.robot.subsystems.DriveTrain;

public class PathweaverImport extends CommandBase {
  private DriveTrain driveTrain;
  private Timer timer;
  private HolonomicDriveController controller;
  private TrajectoryConfig config;
  private Trajectory exampleTrajectory;
  private String traj;
  /** Creates a new ExampleTraj. */
  public PathweaverImport(DriveTrain driveTrain, String traj) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.traj = traj;
    timer = new Timer();
    
    controller = new HolonomicDriveController(
      new PIDController(0, 0, 0), new PIDController(0, 0, 0),
      new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset Drive Encoders
    driveTrain.resetPosition();
    // Create config for trajectory
    config =
        new TrajectoryConfig(PATHFINDER_CONSTANTS.MAX_DRIVE_SPEED_MPS, PATHFINDER_CONSTANTS.MAX_DRIVE_ACCELERATION_MPS);


    // An example trajectory to follow.  All units in meters.
    String trajectoryJSON = "trajPaths/output/ToD5.wpilib.json";
    exampleTrajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Trajectory.State goal = exampleTrajectory.sample(timer.get());
    ChassisSpeeds adjustedSpeeds = controller.calculate(driveTrain.getPose(), goal, Rotation2d.fromDegrees(0));

    driveTrain.setModuleStates(DRIVE_TRAIN_CONSTANTS.DRIVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds));
    // driveTrain.drive(.1, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 20;
  }
}
