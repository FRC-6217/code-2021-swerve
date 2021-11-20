/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommand;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.STATE;
import frc.robot.commands.ArmLiftCommand;
import frc.robot.commands.BallShooterCommandAuto;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.JoyDriveForward;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.Wait;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoWeekZero extends SequentialCommandGroup {
  /**
   * Creates a new AutoWeekZero.
   */
  public AutoWeekZero(BallShooter bs, ShooterIntake si, ArmLift al, DriveTrain ds) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ArmLiftCommand(al, STATE.DOWN, true),
    new Wait(1),
    new ArmLiftCommand(al, STATE.OFF, true),
    new  BallShooterCommandAuto(bs, true), 
    new Wait(3),
    new ShooterIntakeCommand(si, STATE.FORWARDS, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.OFF, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.FORWARDS, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.OFF, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.FORWARDS, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.OFF, true),
    new Wait(1),
    new ShooterIntakeCommand(si, STATE.OFF, true),
    new BallShooterCommandAuto(bs, false),
    new JoyDriveForward(ds, 0.7),
    new Wait(2),
    new JoyDriveForward(ds, 0));
  }
}
