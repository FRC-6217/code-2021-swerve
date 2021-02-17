/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM_LIFT_CONSTANTS;

public class ArmLift extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANDigitalInput leftLimitUp;
  private CANDigitalInput leftLimitDown;
  private CANDigitalInput rightLimitUp;
  private CANDigitalInput rightLimitDown;
  private int leftDirection = 1;
  private int rightDirection = 1;
  
  public ArmLift() {
    leftMotor = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_LEFT, MotorType.kBrushed);
    rightMotor = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_RIGHT, MotorType.kBrushed);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    
    leftLimitUp = leftMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    leftLimitDown = leftMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    rightLimitUp = rightMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rightLimitDown = rightMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    
    if (ARM_LIFT_CONSTANTS.IS_NEGATED_LEFT) {
      leftDirection = -1;
    }
    if (ARM_LIFT_CONSTANTS.IS_NEGATED_RIGHT) {
      rightDirection = -1;
    }
  }

  public void up(){
    leftMotor.set(leftDirection * ARM_LIFT_CONSTANTS.SPEED);
    rightMotor.set(rightDirection * ARM_LIFT_CONSTANTS.SPEED);
  }

  public void down(){
    leftMotor.set(-leftDirection * ARM_LIFT_CONSTANTS.SPEED);
    rightMotor.set(-rightDirection * ARM_LIFT_CONSTANTS.SPEED);
  }

  public void upLimit() {
    if(!leftLimitUp.get()) {
      leftMotor.set(leftDirection * ARM_LIFT_CONSTANTS.SPEED);
    }
    else{
      leftMotor.set(0);
    }

    if(!rightLimitUp.get()){
      rightMotor.set(rightDirection * ARM_LIFT_CONSTANTS.SPEED);
    }
    else{
      rightMotor.set(0);
    }
  }

  public void downLimit() {
    if(!leftLimitDown.get()) {
      leftMotor.set(-leftDirection * ARM_LIFT_CONSTANTS.SPEED);
    }
    else{
      leftMotor.set(0);
    }

    if(!rightLimitDown.get()){
      rightMotor.set(-rightDirection * ARM_LIFT_CONSTANTS.SPEED);
    }
    else{
      rightMotor.set(0);
    }
  }

  public void off() {
      leftMotor.set(0);
      rightMotor.set(0);
  }

  public boolean getUpperLimits(){
    return(leftLimitUp.get() && rightLimitUp.get());
  }

  public boolean getLowerLimits(){
    return(leftLimitDown.get() && rightLimitDown.get());
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Limit UP", leftLimitUp.get());
    SmartDashboard.putBoolean("Left Limit DOWN", leftLimitDown.get());
    SmartDashboard.putBoolean("Right Limit UP", rightLimitUp.get());
    SmartDashboard.putBoolean("Right Limit DOWN", rightLimitDown.get());
  }
}
