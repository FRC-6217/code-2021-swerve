// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.WHEEL_DRIVE_CONSTANTS;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final VictorSPX turningMotor;

  private final CANEncoder driveEncoder;
  private final CANAnalog turningEncoder;

  private final CANPIDController drivePIDController;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderPorts,
      int[] turningEncoderPorts,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new VictorSPX(turningMotorChannel);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = driveMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
    turningEncoder.getPosition()

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPositionConversionFactor(3); //TODO - fix this

    // Set whether drive encoder should be reversed or not
    driveEncoder.setInverted(driveEncoderReversed);

    drivePIDController = driveMotor.getPIDController();
    drivePIDController.setP(0.5);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

    private double getAngle() {
        double value = turningEncoder.getVoltage();
        double m = (360)/(WHEEL_DRIVE_CONSTANTS.MAX_VOLTAGE - WHEEL_DRIVE_CONSTANTS.MIN_VOLTAGE);
        double b = 180 - (m * WHEEL_DRIVE_CONSTANTS.MAX_VOLTAGE);
        return value * m + b;
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getVoltage()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        turningPIDController.calculate(turningEncoder.get(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turningMotor.set(turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.reset();
    turningEncoder.reset();
  }
}