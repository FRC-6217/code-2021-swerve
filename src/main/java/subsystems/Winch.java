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
import frc.robot.Constants.WINCH_CONSTANTS;

public class Winch extends SubsystemBase {
  private CANSparkMax winch;
  private int direction = 1;

  public Winch() {
    winch = new CANSparkMax(WINCH_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    
    winch.restoreFactoryDefaults();
    winch.setIdleMode(IdleMode.kBrake);

    if (WINCH_CONSTANTS.IS_NEGATED) {
      direction = -1;
    }
  }

  public void up() {
    winch.set(direction*WINCH_CONSTANTS.SPEED);
  }

  public void down() {
    winch.set(-direction*WINCH_CONSTANTS.SPEED);
  }

  public void off() {
    winch.set(0);
  }

  @Override
  public void periodic() {
  }
}
