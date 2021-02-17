/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER_INTAKE_CONSTANTS;

public class ShooterIntake extends SubsystemBase {
  VictorSPX shooterIntakeMotor;
  int direction = 1;
  
  public ShooterIntake() {
    shooterIntakeMotor = new VictorSPX(SHOOTER_INTAKE_CONSTANTS.MOTOR_CONTROLLER_ID);
    if (SHOOTER_INTAKE_CONSTANTS.IS_NEGATED) {
      direction = -1;
    }
  }

  public void forward() {
    shooterIntakeMotor.set(ControlMode.PercentOutput, direction*SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  public void reverse() {
    shooterIntakeMotor.set(ControlMode.PercentOutput, -direction*SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  public void off() {
    shooterIntakeMotor.set(ControlMode.PercentOutput, 0);
  }
  
  @Override
  public void periodic() {
  }
}
