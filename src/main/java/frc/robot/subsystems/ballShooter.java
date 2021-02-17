/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BALL_SHOOTER_CONSTANTS;

public class ballShooter extends SubsystemBase {

  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  private int topDirection = 1;
  private int bottomDirection = 1;
  private CANEncoder topEncoder;
  private CANEncoder bottomEncoder;

  private CANPIDController topPID;
  private CANPIDController bottomPID;
  private double topRPM = 0;
  private double bottomRPM = 0;
  private double kP;
  private double kI;
  private double kD;
  private double kIz;
  private double kFF;
  private double kMinOutput;
  private double kMaxOutput;

  public ballShooter() {
    topMotor = new CANSparkMax(BALL_SHOOTER_CONSTANTS.MOTOR_CONTROLLER_ID_TOP, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(BALL_SHOOTER_CONSTANTS.MOTOR_CONTROLLER_ID_BOTTOM, MotorType.kBrushless);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    if (BALL_SHOOTER_CONSTANTS.IS_NEGATED_TOP) {
      topDirection = -1;
    }
    if (BALL_SHOOTER_CONSTANTS.IS_NEGATED_BOTTOM) {
      bottomDirection = -1;
    }

    //Shooter Encoders
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    //Set PID constants
    kP = BALL_SHOOTER_CONSTANTS.KP;
    kI = BALL_SHOOTER_CONSTANTS.KI;
    kD = BALL_SHOOTER_CONSTANTS.KD;
    kIz = BALL_SHOOTER_CONSTANTS.KIZ;
    kFF = BALL_SHOOTER_CONSTANTS.KFF;
    kMinOutput = BALL_SHOOTER_CONSTANTS.KMINOUTPUT;
    kMaxOutput = BALL_SHOOTER_CONSTANTS.KMAXOUTPUT;

    if(BALL_SHOOTER_CONSTANTS.ENABLE_TUNING){
      enableTuning();
    }

    //Shooter PIDs
    topPID = topMotor.getPIDController();
    bottomPID = bottomMotor.getPIDController();
    setPID(topPID);
    setPID(bottomPID);
  }

  //Activate Shooter
  public void on(){
    topMotor.set(topDirection * BALL_SHOOTER_CONSTANTS.SPEED);
    bottomMotor.set(bottomDirection * BALL_SHOOTER_CONSTANTS.SPEED);
  }

  //Activate Shooter with PID
  //Input RPM
  public void onPID(double topRPM, double bottomRPM) {
    topPID.setReference(topDirection * topRPM, ControlType.kVelocity);
    bottomPID.setReference(bottomDirection * bottomRPM, ControlType.kVelocity);
  }

  //Deactivate Shooter
  public void off() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public void enableTuning(){
    SmartDashboard.putNumber("KpShooter", kP);
    SmartDashboard.putNumber("KiShooter", kI);
    SmartDashboard.putNumber("KdShooter", kD);
    SmartDashboard.putNumber("IzShooter", kIz);
    SmartDashboard.putNumber("FfShooter", kFF);
    SmartDashboard.putNumber("MaxInShooter", kMaxOutput);
    SmartDashboard.putNumber("MinInShooter", kMinOutput);
  }

  public void updatePID(){
    boolean isUpdated = false;

    if(SmartDashboard.getNumber("KpShooter", 0) != kP){
      kP = SmartDashboard.getNumber("KpShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KiShooter", 0) != kI){
      kI = SmartDashboard.getNumber("KiShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KdShooter", 0) != kD){
      kD = SmartDashboard.getNumber("KdShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("IzShooter", 0) != kIz){
      kIz = SmartDashboard.getNumber("IzShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("FfShooter", 0) != kFF){
      kFF = SmartDashboard.getNumber("FfShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("MaxInShooter", 0) != kMaxOutput){
      kMaxOutput = SmartDashboard.getNumber("MaxInShooter", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("MinInShooter", 0) != kMinOutput){
      kMinOutput = SmartDashboard.getNumber("MinInShooter", 0);
      isUpdated = true;  
    }

    if(isUpdated){
      setPID(topPID);
      setPID(bottomPID);
    }
  }

  private void setPID(CANPIDController pid) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Top", topMotor.getOutputCurrent());
    SmartDashboard.putNumber("Current Bottom", bottomMotor.getOutputCurrent());
    //Print Current Velocity of Both Motors
    SmartDashboard.putNumber("Top RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom RPM", bottomEncoder.getVelocity());

    if(BALL_SHOOTER_CONSTANTS.ENABLE_TUNING){
      updatePID();
    }
  }
}
