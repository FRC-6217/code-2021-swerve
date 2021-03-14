/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.STATE;
import frc.robot.commands.Align;
import frc.robot.commands.ArmLiftCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.ColorWheelCommand;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.NotShooterIntakeCommand;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.autoCommand.Test;
import frc.robot.libraries.Angle;
import frc.robot.libraries.Distance;
import frc.robot.libraries.FakeJoystick;
import frc.robot.libraries.JoystickTrigger;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NotShooterIntake;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Joysticks
  private final FakeJoystick fake = new FakeJoystick();

  private final Joystick joy = new Joystick(Constants.DRIVESTICK_PORT);
  private final XboxController xbox = new XboxController(Constants.XBOX_PORT);

  //Helper classes
  private final Angle angle = new Angle();
  private final Distance distance = new Distance();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final ArmLift armLift = new ArmLift();
  private final ShooterIntake shooterIntake = new ShooterIntake();
  private final NotShooterIntake notShooterIntake = new NotShooterIntake();
  private final BallShooter ballShooter = new BallShooter();
  private final Winch winch = new Winch();
  private final ColorWheel colorWheel = new ColorWheel();
  private final LimeLight limeLight = new LimeLight(angle, distance);


  public RobotContainer() {
    // Configure the button bindings
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new JoyDrive(driveTrain, joy));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  //  new JoystickButton(driveStick, 1).whileHeld(new Align(DriveTrain, driveStick, angle));
 
    //Xbox Joystick
  
    new JoystickTrigger(xbox, 3).whileHeld(new ShooterIntakeCommand(shooterIntake, STATE.FORWARDS));
    new JoystickButton(xbox, Button.kBumperLeft.value).whileHeld(new ArmLiftCommand(armLift, STATE.UP));
    new JoystickTrigger(xbox, 2).whileHeld(new ArmLiftCommand(armLift, STATE.DOWN));
    new JoystickButton(xbox, Button.kBumperRight.value).toggleWhenPressed(new BallShooterCommand(ballShooter, true));
    new JoystickButton(xbox, Button.kY.value).whileHeld(new WinchCommand(winch, STATE.UP));
    new JoystickButton(xbox, Button.kX.value).whileHeld(new WinchCommand(winch, STATE.DOWN));
    new JoystickButton(xbox, Button.kA.value).whileHeld(new NotShooterIntakeCommand(notShooterIntake, STATE.FORWARDS));
    new JoystickButton(xbox, Button.kB.value).whileHeld(new NotShooterIntakeCommand(notShooterIntake, STATE.REVERSE));
    new JoystickButton(xbox, Button.kStickRight.value).whileHeld(new ColorWheelCommand(colorWheel));
  

    // arm hn
    /*
    new JoystickButton(xbox, 1).whileHeld(new ArmLiftSeperate(armLift, SIDE.LEFT, STATE.UP));
    new JoystickButton(xbox, 2).whileHeld(new ArmLiftSeperate(armLift, SIDE.LEFT, STATE.DOWN));
    new JoystickButton(xbox, 3).whileHeld(new ArmLiftSeperate(armLift, SIDE.RIGHT, STATE.UP));
    new JoystickButton(xbox, 4).whileHeld(new ArmLiftSeperate(armLift, SIDE.RIGHT, STATE.DOWN));


    new JoystickButton(xbox, 5).whileHeld(new ArmLiftSeperate(armLift, SIDE.BOTH, STATE.UP));
    new JoystickButton(xbox, 6).whileHeld(new ArmLiftSeperate(armLift, SIDE.BOTH, STATE.DOWN));
    */
}



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // fake.setLeftShift(.9);
    // return new Test(DriveTrain, fake);
    return null;
    // An ExampleCommand will run in autonomous
  //  return new AutoWeekZero(ballShooter, shooterIntake);
  // return null;
  }
}
