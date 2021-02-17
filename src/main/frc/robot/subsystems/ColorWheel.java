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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.COLOR_WHEEL_CONSTANTS;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class ColorWheel extends SubsystemBase {

  private CANSparkMax motor;
  private int direction = 1;

  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatch;

  private final Color blue;
  private final Color red;
  private final Color yellow;
  private final Color green;
  
  public ColorWheel() {
    motor = new CANSparkMax(Constants.COLOR_WHEEL_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    if (COLOR_WHEEL_CONSTANTS.IS_NEGATED) {
      direction = -1;
    }

    colorSensor = new ColorSensorV3(Constants.COLOR_WHEEL_CONSTANTS.PORT);
    colorMatch = new ColorMatch();

    blue = ColorMatch.makeColor(Constants.COLORS_CONSTANTS.RGB_FRC_BLUE[0], Constants.COLORS_CONSTANTS.RGB_FRC_BLUE[1], Constants.COLORS_CONSTANTS.RGB_FRC_BLUE[2]);
    red = ColorMatch.makeColor(Constants.COLORS_CONSTANTS.RBG_FRC_RED[0], Constants.COLORS_CONSTANTS.RBG_FRC_RED[1], Constants.COLORS_CONSTANTS.RBG_FRC_RED[2]);
    yellow = ColorMatch.makeColor(Constants.COLORS_CONSTANTS.RGB_FRC_YELLOW[0], Constants.COLORS_CONSTANTS.RGB_FRC_YELLOW[1], Constants.COLORS_CONSTANTS.RGB_FRC_YELLOW[2]);
    green = ColorMatch.makeColor(Constants.COLORS_CONSTANTS.RBG_FRC_GREEN[0], Constants.COLORS_CONSTANTS.RBG_FRC_GREEN[1], Constants.COLORS_CONSTANTS.RBG_FRC_GREEN[2]);

    colorMatch.addColorMatch(blue);
    colorMatch.addColorMatch(red);
    colorMatch.addColorMatch(yellow);
    colorMatch.addColorMatch(green);
  }

  public void forwards() {
   motor.set(direction * Constants.COLOR_WHEEL_CONSTANTS.SPEED);
  }

  public void backwards() {
    motor.set(-direction * Constants.COLOR_WHEEL_CONSTANTS.SPEED);
  }

  public void off() {
   motor.set(0);
  }

  public Color getColor(){
    return colorSensor.getColor();
  }

  public Color getClosestColor(){
    return colorMatch.matchClosestColor(getColor()).color;
  }

  public double getConfidence(){
    return colorMatch.matchClosestColor(getColor()).confidence;
  }

  public int getProximity(){
    return colorSensor.getProximity();
  }
  
  public Color getBlue(){
    return blue;
  }
  public Color getGreen(){
    return green;
  }
  public Color getYellow(){
    return yellow;
  }
  public Color getRed(){
    return red;
  }

  @Override
  public void periodic() {
    String colorString;

    if (getClosestColor() == blue) {
      colorString = "Blue";
    } else if (getClosestColor() == red) {
      colorString = "Red";
    } else if (getClosestColor() == green) {
      colorString = "Green";
    } else if (getClosestColor() == yellow) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", getColor().red);
    SmartDashboard.putNumber("Green", getColor().green);
    SmartDashboard.putNumber("Blue", getColor().blue);
    SmartDashboard.putNumber("Confidence", getConfidence());
    SmartDashboard.putString("Detected Color", colorString);
  }
}
