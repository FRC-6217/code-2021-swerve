/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ALIGN_COMMAND_CONSTANTS;
import frc.robot.libraries.Angle;
import frc.robot.libraries.Distance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class Align extends CommandBase {

  private final DriveTrain driveTrain;
  private LimeLight light;
  private Joystick joy;
  private double y;
  private double x;
  private double z;
  private Angle angle;
  private Distance distance;

  private boolean angleUse;
  private boolean distanceUse;

  private PIDController pidZ;
  private double errorZ;
  private double outputZ;

  private double kPZ = ALIGN_COMMAND_CONSTANTS.kPZ;
  private double kIZ = ALIGN_COMMAND_CONSTANTS.kIZ;
  private double kDZ = ALIGN_COMMAND_CONSTANTS.kDZ;

  private PIDController pidY;
  private double errorY;
  private double outputY;

  private double kPY = ALIGN_COMMAND_CONSTANTS.kPY;
  private double kIY = ALIGN_COMMAND_CONSTANTS.kIY;
  private double kDY = ALIGN_COMMAND_CONSTANTS.kDY;

  private boolean atSetZ;
  private boolean atSetY;
  
  public Align(DriveTrain train, Joystick joy, Angle angle, Distance distance) {
    addRequirements(train);

    driveTrain = train;
    this.joy = joy;
    this.angle = angle;
    this.distance = distance;

    angleUse = true;
    distanceUse = true;

    pidZ = new PIDController(kPZ, kIZ, kDZ);
    pidY = new PIDController(kPY, kIY, kDY);

    pidZ.setTolerance(0.1, 0.1);
    pidY.setTolerance(0.1, 0.1);

    SmartDashboard.putNumber("KpAlignZ", 0.1);
    SmartDashboard.putNumber("KiAlignZ", 0);
	  SmartDashboard.putNumber("KdAlignZ", 0);
	
	  SmartDashboard.putNumber("KpAlignY", 0.05);
    SmartDashboard.putNumber("KiAlignY", 0);
    SmartDashboard.putNumber("KdAlignY", 0);
    
  }

  public Align(DriveTrain train, Joystick joy, Angle angle) {
    addRequirements(train);

    driveTrain = train;
    this.joy = joy;
    this.angle = angle;

    angleUse = true;
    distanceUse = false;

    pidZ = new PIDController(kPZ, kIZ, kDZ);
    pidZ.setTolerance(10, 10);
  }

  public Align(DriveTrain train, LimeLight light, Joystick joy, Distance distance) {
    addRequirements(train);

    this.light = light;
    light.limeRequired();

    driveTrain = train;
    this.joy = joy;
    this.distance = distance;

    angleUse = false;
    distanceUse = true;

    atSetY = false;
    pidY = new PIDController(kPY, kIY, kDY);
    pidY.setTolerance(50, 1000);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  

    atSetZ = false;
    
    double localAngle = angle.getAngle();
    
    errorZ = pidZ.calculate(localAngle, 0);
    SmartDashboard.putNumber("ErrorZ", errorZ);
    outputZ = MathUtil.clamp(errorZ, -1, 1);

    if(pidZ.atSetpoint()){
      atSetZ = true;
    }

    atSetY = false;
    
    double localDistance = distance.getDistance();
    
    errorY = pidY.calculate(localDistance, 0);
    SmartDashboard.putNumber("ErrorY", errorY);
    outputY = MathUtil.clamp(errorY, -1, 1);

    if(pidY.atSetpoint()){
      atSetY = true;
    }

  }

  private double fit(double value, double minInput, double maxInput, double minOutput, double maxOutput){
    double m = (maxOutput - minOutput)/(maxInput - minInput);
    double b = maxOutput - (m * maxInput);
    return value * m + b;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Dead zone
    x = (Math.abs(joy.getRawAxis(1)) > .2) ? joy.getRawAxis(0) : 0.0;
    y = (Math.abs(joy.getRawAxis(0)) > .2) ? joy.getRawAxis(1) : 0.0;
    z = (Math.abs(joy.getRawAxis(2)) > .2) ? joy.getRawAxis(2) : 0.0;

    if(angleUse && !distanceUse){
      updatePIDZ();
  
      double localAngle = angle.getAngle();
      if((localAngle != (Double.POSITIVE_INFINITY))){
        if (pidZ.atSetpoint()) {
          localAngle = 0;
        }      

        errorZ = pidZ.calculate(localAngle, -1);
        SmartDashboard.putNumber("ErrorZ", errorZ);
        outputZ = MathUtil.clamp(errorZ, -1, 1);

        driveTrain.drive(x, y, -outputZ, false);
        //TODO set max speed constant
      }
      else{
        driveTrain.drive(x, y, z, false);
      }
    }
  
    if(!angleUse && distanceUse){
      updatePIDY();

      double localDistance = distance.getDistance();
      if((localDistance != (Double.POSITIVE_INFINITY))){
        SmartDashboard.putBoolean("At Set", pidY.atSetpoint());
        SmartDashboard.putNumber("Velocity", pidY.getVelocityError());
        SmartDashboard.putNumber("Position", pidY.getPositionError());
        
        if (atSetY) {
          localDistance = 7;
        }      

        errorY = pidY.calculate(localDistance, 100);
        SmartDashboard.putNumber("ErrorY", errorY);
        fit(outputY, -60, 60, -1, 1);
        outputY = MathUtil.clamp(errorY, -1, 1);

        atSetY = false;
        if(pidY.atSetpoint()){
          atSetY = true;
        }

        driveTrain.drive(outputY * 0.5, y, z, false);
        //TODO set max speed constant
      }
      else{
        driveTrain.drive(x, y, z, false);
      }
    }

    if(angleUse && distanceUse){
      updatePIDZ();
      updatePIDY();

      double localAngle = angle.getAngle();
      double localDistance = distance.getDistance();
      if((localAngle != (Double.POSITIVE_INFINITY)) && (localDistance != (Double.POSITIVE_INFINITY))){
        if (pidZ.atSetpoint()) {
          localAngle = 0;
        }
        if (pidY.atSetpoint()) {
          localDistance = 200;
        }      

        errorZ = pidZ.calculate(localAngle, 0);
        SmartDashboard.putNumber("ErrorZ", errorZ);
        outputZ = MathUtil.clamp(errorZ, -1, 1);

        errorY = pidY.calculate(localDistance, 100);
        SmartDashboard.putNumber("ErrorY", errorY);
        fit(outputY, -60, 60, -1, 1);
        outputY = MathUtil.clamp(errorY, -0.5, 0.5);

        driveTrain.drive(outputY * 0.5, -y, -outputZ * 0.5, false);
        //TODO set max speed constant
      }
      else{
        driveTrain.drive(x, y, z, false);
      }

    }

  }

  public void updatePIDZ(){
    boolean isUpdated = false;
  
    if(SmartDashboard.getNumber("KpAlignZ", 0) != kPZ){
      kPZ = SmartDashboard.getNumber("KpAlignZ", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KiAlignZ", 0) != kIZ){
      kIZ = SmartDashboard.getNumber("KiAlignZ", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KdAlignZ", 0) != kDZ){
      kDZ = SmartDashboard.getNumber("KdAlignZ", 0);
      isUpdated = true;  
    }

    if(isUpdated){
      pidZ.setP(kPZ);
      pidZ.setI(kIZ);
      pidZ.setD(kDZ);
    }
  }

  public void updatePIDY(){
    boolean isUpdated = false;
    
    if(SmartDashboard.getNumber("KpAlignY", 0) != kPY){
      kPY = SmartDashboard.getNumber("KpAlignY", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KiAlignY", 0) != kIY){
      kIY = SmartDashboard.getNumber("KiAlignY", 0);
      isUpdated = true;  
    }
    if(SmartDashboard.getNumber("KdAlignY", 0) != kDY){
      kDY = SmartDashboard.getNumber("KdAlignY", 0);
      isUpdated = true;  
    }

    if(isUpdated){
      pidY.setP(kPY);
      pidY.setI(kIY);
      pidY.setD(kDY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrain.Drive(0, 0, 0, 0);
    // light.limeNotRequired();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(angleUse && distanceUse){
    //   return (pidZ.atSetpoint() && pidY.atSetpoint());
    // }
    // else if(angleUse && !distanceUse){
    //   return (pidZ.atSetpoint());
    // }
    // else if(!angleUse && distanceUse){
    //   return (pidY.atSetpoint());
    // }
    // else{
    //   return true;
    // }
    return true;
  }

  public String toString(){
    return "AlignZ";
  }
}
