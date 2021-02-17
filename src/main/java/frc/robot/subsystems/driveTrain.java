/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.WheelDrive;
import frc.robot.Constants.DRIVE_TRAIN_CONSTANTS;

public class driveTrain extends SubsystemBase {
  	
	//constants for the width and length;
	private final double L = DRIVE_TRAIN_CONSTANTS.LENGTH; //front to back in.
	private final double W = DRIVE_TRAIN_CONSTANTS.WIDTH; //Left to right in.

	//Wheel Drive Objetcs
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;

	//Gyro object
	private Gyro gyro;

	//Transform variables
	private double x1;
	private double y1;

  	public driveTrain() {

		//Wheel Drive Modules
		backRight = new WheelDrive(DRIVE_TRAIN_CONSTANTS.BR_SPEED_MOTOR, DRIVE_TRAIN_CONSTANTS.BR_ANGLE_MOTOR);
		backLeft = new WheelDrive(DRIVE_TRAIN_CONSTANTS.BL_SPEED_MOTOR, DRIVE_TRAIN_CONSTANTS.BL_ANGLE_MOTOR);
		frontRight = new WheelDrive(DRIVE_TRAIN_CONSTANTS.FR_SPEED_MOTOR, DRIVE_TRAIN_CONSTANTS.FR_ANGLE_MOTOR);
		frontLeft = new WheelDrive(DRIVE_TRAIN_CONSTANTS.FL_SPEED_MOTOR, DRIVE_TRAIN_CONSTANTS.FL_ANGLE_MOTOR);
		
		gyro = new ADXRS450_Gyro();
		resetGyro();
 	}

	public double TransformX(double x, double y, boolean isReversed){
		if(isReversed){
			x1 = (x * Math.cos((GetAngle() + 180) * (Math.PI / 180))) - (y * Math.sin((GetAngle() + 180) * (Math.PI / 180)));	
		}
		else{
			x1 = (x * Math.cos(GetAngle() * (Math.PI / 180))) - (y * Math.sin(GetAngle() * (Math.PI / 180)));
		}
		return x1;
	}

	public double TransformY(double x, double y, boolean isReversed){
		if(isReversed){
			y1 = (x * Math.sin((GetAngle() + 180) * (Math.PI / 180))) + (y * Math.cos((GetAngle() + 180) * (Math.PI / 180)));
		}
		else{
			y1 = (x * Math.sin(GetAngle() * (Math.PI / 180))) + (y * Math.cos(GetAngle() * (Math.PI / 180)));
		}
		return y1;
	}

	public void Drive(double x, double y, double z, double governer) {
		x *= governer;
		y *= governer;
		z *= governer;
		SmartDashboard.putNumber("xSpeed", x);
		SmartDashboard.putNumber("ySpeed", y);
		SmartDashboard.putNumber("zSpeed", z);

        /*
		 * First we need to find the radius of the circle the robot is going to spin and
		 * for that we use the Pythagorean theorem and y *= -1 means y is assigned the
		 * value of y * -1
		 */
		double r = Math.sqrt((L * L) + (W * W));
		y *= -1;
		// The next thing we do is assign the value a to the equation. a makes the robot
		// go backwards
		// Note: The L/r part makes the code in radians not degrees

		// a makes it go backwards;
		double a = x - z * (L / r);
		// b makes it go forwards.
		double b = x + z * (L / r);
		// c makes it go left.
		double c = y - z * (W / r);
		// d makes it go right.
		double d = y + z * (W / r);

		/*
		 * Now we do the Pythagorean theorem again because the robot will only go in a
		 * straight line so we find the hypotenuse using our above variables for all
		 * direction combos.
		 */
		double backRightSpeed = Math.sqrt((a * a) + (d * d));
		double backLeftSpeed = Math.sqrt((a * a) + (c * c));
		double frontRightSpeed = Math.sqrt((b * b) + (d * d));
		double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

		/*
		 * In order to find the angles we need to turn based on the inputs of the
		 * controller the code turns the tangent of the coordinates (x,y) into
		 * (radius,angle) to find the angle that applies to our robot. Then it's divided
		 * by pi to turn from radians into degrees.
		 */
		double backLeftAngle = Math.atan2(a, c) / Math.PI;
		double backRightAngle = Math.atan2(a, d) / Math.PI;
		double frontRightAngle = Math.atan2(b, d) / Math.PI;
		double frontLeftAngle = Math.atan2(b, c) / Math.PI;

		/*
		 * Lastly the results of the above code are all plugged back in to be used
		 * later.
		 */
		backRight.drive(backRightSpeed , backRightAngle);
		backLeft.drive(-backLeftSpeed , backLeftAngle);
		frontRight.drive(frontRightSpeed , frontRightAngle);
		frontLeft.drive(frontLeftSpeed , frontLeftAngle);
	}
	
	public void resetGyro(){ //had to change from ResetGryo to ResetGyro so if that wasn´t something I was supposed to change feel free to change it back
		gyro.reset();
	}

	public void calibrateGyro(){
		gyro.calibrate();
	}

	public double GetAngle(){
		SmartDashboard.putNumber("Gyro", -gyro.getAngle());
		return -gyro.getAngle();
	}

	@Override
  	public void periodic() {
		if(CommandScheduler.getInstance().requiring(this) != null)
		SmartDashboard.putString("DriveCommand", CommandScheduler.getInstance().requiring(this).toString());
  	}
}
