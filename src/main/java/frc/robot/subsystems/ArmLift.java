// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM_LIFT_CONSTANTS;



public class ArmLift extends SubsystemBase {
  // varibles for motors
  private CANSparkMax left;
  private CANSparkMax right;

  // varibles for limit switches
  private CANDigitalInput upLeftLimit;
  private CANDigitalInput upRightLimit;
  private CANDigitalInput downLeftLimit;
  private CANDigitalInput downRightLimit;


  /** Creates a new ArmLift. */
  public ArmLift() {
    // creates the motor controller objects
    left = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_LEFT, MotorType.kBrushed);
    right = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_RIGHT, MotorType.kBrushed);

    // creates the limit switch objects
    upLeftLimit = left.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    upRightLimit = right.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    downLeftLimit = left.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    downRightLimit = right.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    // restores factory defult
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    // tells the wheels to stop 
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);

    // set motors inverted when negated
    if(ARM_LIFT_CONSTANTS.IS_NEGATED_LEFT) {
      left.setInverted(true);
    }
    if(ARM_LIFT_CONSTANTS.IS_NEGATED_RIGHT) {
      right.setInverted(true);
    }
  }

  //sensor methods
  public boolean getUpLeftLimit() {
    return upLeftLimit.get();
  }

  public boolean getUpRightLimit() {
    return upRightLimit.get();
  }

  public boolean getDownLeftLimit() {
    return downLeftLimit.get();
  }

  public boolean getDownRightLimit() {
    return downRightLimit.get();
  }

  // motor control methods

  // up motors
  public void upLeftMotor() {
    left.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  public void upRightMotor() {
    right.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  // down motors
  public void downLeftMotor() {
    left.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  public void downRightMotor() {
    right.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  // off motors
  public void offLeftMotor() {
    left.set(0);
  }
  
  public void offRightMotor() {
    right.set(0);
  }
  
  // calling previous functions
  public void upMotor() {
    upLeftMotor();
    upRightMotor();
  }

  public void downMotor() {
    downLeftMotor();
    downRightMotor();
  }
  
  public void offMotor() {
    offLeftMotor();
    offRightMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
