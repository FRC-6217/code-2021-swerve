/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NOT_SHOOTER_INTAKE_CONSTANTS;


public class NotShooterIntake extends SubsystemBase {
  private CANSparkMax motor;
  private int direction = 1;
  
  public NotShooterIntake() {
    motor = new CANSparkMax(Constants.NOT_SHOOTER_INTAKE_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    if (NOT_SHOOTER_INTAKE_CONSTANTS.IS_NEGATED) {
      direction = -1;
    }
  }

  public void suck() {
    motor.set(direction * NOT_SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  public void spit() {
    motor.set(-direction * NOT_SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  public void off() {
    motor.set(0);
  }

  @Override
  public void periodic() {
  }
}
