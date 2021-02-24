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
  private boolean getUpLeftLimit() {
    return upLeftLimit.get();
  }

  private boolean getUpRightLimit() {
    return upRightLimit.get();
  }

  private boolean getDownLeftLimit() {
    return downLeftLimit.get();
  }

  private boolean getDownRightLimit() {
    return downRightLimit.get();
  }

  // motor control methods

  // up motors
  private void upLeftMotor() {
    left.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  private void upRightMotor() {
    right.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  // down motors
  private void downLeftMotor() {
    left.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  private void downRightMotor() {
    right.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  // off motors
  private void offLeftMotor() {
    left.set(0);
  }
  
  private void offRightMotor() {
    right.set(0);
  }
  
  // calling previous functions
  public void offMotor() {
    offLeftMotor();
    offRightMotor();
  }


  //This is the limit switch for the arm
  private void upLeftLimited() {
    if(!getUpLeftLimit()) {
      upLeftMotor();
    }

    else {
      offLeftMotor();
    }
  }


  private void upRightLimited() {
    if(!getUpRightLimit()) {
      upRightMotor();
    }

    else {
      offRightMotor();
    }
  }


  private void downLeftLimited() {
    if(!getDownLeftLimit()) {
      downLeftMotor();
    }

    else {
      offLeftMotor();
    }
  }


  private void downRightLimited() {
    if(!getDownRightLimit()) {
      downRightMotor();
    }

    else {
      offRightMotor();
    }
  }

  public boolean upMotorLimited() {
    upLeftLimited();
    upRightLimited();

    return getUpLeftLimit() && getUpRightLimit();
  }

  public boolean downMotorLimited() {
    downLeftLimited();
    downRightLimited();

    return getDownLeftLimit() && getDownRightLimit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
