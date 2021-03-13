package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVE_TRAIN_CONSTANTS;
import frc.robot.Constants.DRIVE_TRAIN_CONSTANTS.MODULE;
import frc.robot.libraries.SwerveModule;

/* DriveTrain */
public class DriveTrain extends SubsystemBase {
  //Map object to consolidate swerve module objects
  Map<MODULE, SwerveModule> module = new EnumMap<>(MODULE.class); 

  // Define Robot swerve modules
  private final SwerveModule frontLeft =
    new SwerveModule(
      DRIVE_TRAIN_CONSTANTS.FL_ANGLE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.FL_DRIVE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.FL_ANGLE_CURRENT_CHANNEL,
      DRIVE_TRAIN_CONSTANTS.FL_DRIVE_CURRENT_CHANNEL,
      false,
      true);

  private final SwerveModule frontRight =
    new SwerveModule(
      DRIVE_TRAIN_CONSTANTS.FR_ANGLE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.FR_DRIVE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.FR_ANGLE_CURRENT_CHANNEL,
      DRIVE_TRAIN_CONSTANTS.FR_DRIVE_CURRENT_CHANNEL,
      false,
      true);

  private final SwerveModule backLeft = 
    new SwerveModule(
      DRIVE_TRAIN_CONSTANTS.BL_ANGLE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.BL_DRIVE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.BL_ANGLE_CURRENT_CHANNEL,
      DRIVE_TRAIN_CONSTANTS.BL_DRIVE_CURRENT_CHANNEL,
      false,
      false);

  private final SwerveModule backRight =
    new SwerveModule(
      DRIVE_TRAIN_CONSTANTS.BR_ANGLE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.BR_DRIVE_MOTOR_ID,
      DRIVE_TRAIN_CONSTANTS.BR_ANGLE_CURRENT_CHANNEL,
      DRIVE_TRAIN_CONSTANTS.BR_DRIVE_CURRENT_CHANNEL,
      false,
      true);

  // Define gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // Define Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DRIVE_TRAIN_CONSTANTS.DRIVE_KINEMATICS, gyro.getRotation2d());

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    module.put(MODULE.FRONT_LEFT, frontLeft);
    module.put(MODULE.FRONT_RIGHT, frontRight);
    module.put(MODULE.BACK_LEFT, backLeft);
    module.put(MODULE.BACK_RIGHT, backRight);
  }


  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DRIVE_TRAIN_CONSTANTS.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the current draw of a specific wheel module's turning motor in amps
   * 
   * @param module The wheel module to return the turning motor current draw of
   * @return The current draw of the wheel module's turning motor in amps
   */
  public double getWheelTurnCurrent(MODULE module){
    return this.module.get(module).getTurningCurrent();
  }

  /**
   * Returns the total current draw of the wheel modules' turning motors in amps
   * 
   * @return The total current draw of the wheel modules' turning motors in amps
   */
  public double getWheelTurnCurrent(){
    return (getWheelTurnCurrent(MODULE.BACK_LEFT) +
            getWheelTurnCurrent(MODULE.BACK_RIGHT) +
            getWheelTurnCurrent(MODULE.FRONT_LEFT) +
            getWheelTurnCurrent(MODULE.FRONT_RIGHT));
  }

  /**
   * Returns the heading of a specific wheel module in radians
   * 
   * @param module The wheel module to return the heading of
   * @return The heading of the wheel module in radians
   */
  public double getWheelHeading(MODULE module){
    return this.module.get(module).getAngle();
  }

  /**
   * Returns the average heading of the wheel modules in radians
   * 
   * @return The average heading of the wheel modules in radians
   */
  public double getWheelHeading(){
    return (getWheelHeading(MODULE.BACK_LEFT) +
            getWheelHeading(MODULE.BACK_RIGHT) +
            getWheelHeading(MODULE.FRONT_LEFT) +
            getWheelHeading(MODULE.FRONT_RIGHT))
            /(4);
  }

  /**
   * Returns the angular velocity of a specific wheel module in radians per second
   * 
   * @param module The wheel module to return the angular velocity of
   * @return The angular velocity of the wheel module in radians per second
   */
  public double getWheelTurnRate(MODULE module){
    return this.module.get(module).getAngularVelocity();
  }

  /**
   * Returns the average angular velocity of the wheel modules in radians per second
   * 
   * @return The average angular velocity of the wheel modules in radians per second
   */
  public double getWheelTurnRate(){
    return (getWheelTurnRate(MODULE.BACK_LEFT) +
            getWheelTurnRate(MODULE.BACK_RIGHT) +
            getWheelTurnRate(MODULE.FRONT_LEFT) +
            getWheelTurnRate(MODULE.FRONT_RIGHT))
            /(4);
  }

  /** Resets a specific wheel module's drive encoder to read a position of 0. */
  public void resetPosition(MODULE module){
    this.module.get(module).resetPosition();
  }

  /** Resets the drive encoders to read a position of 0. */
  public void resetPosition(){
    resetPosition(MODULE.BACK_LEFT);
    resetPosition(MODULE.BACK_RIGHT);
    resetPosition(MODULE.FRONT_LEFT);
    resetPosition(MODULE.FRONT_RIGHT);
  }

  /**
   * Returns the current draw of a specific wheel module's drive motor in amps
   * 
   * @param module The wheel module to return the drive motor current draw of
   * @return The current draw of the wheel module's drive motor in amps
   */
  public double getDriveCurrent(MODULE module){
    return this.module.get(module).getDriveCurrent();
  }

  /**
   * Returns the total current draw of the wheel modules' drive motors in amps
   * 
   * @return The total current draw of the wheel modules' drive motors in amps
   */
  public double getDriveCurrent(){
    return (getDriveCurrent(MODULE.BACK_LEFT) +
            getDriveCurrent(MODULE.BACK_RIGHT) +
            getDriveCurrent(MODULE.FRONT_LEFT) +
            getDriveCurrent(MODULE.FRONT_RIGHT));
  }

  /**
   * Returns the position of a specific wheel module in feet
   * 
   * @param module The wheel module to return the position of
   * @return The position of the wheel module in feet
   */
  public double getPosition(MODULE module){
    return this.module.get(module).getPosition();
  }

  /**
   * Returns the average position of the wheel in feet
   * 
   * @return The average position of the wheel modules in feet
   */
  public double getPosition(){
    return (getPosition(MODULE.BACK_LEFT) +
            getPosition(MODULE.BACK_RIGHT) +
            getPosition(MODULE.FRONT_LEFT) +
            getPosition(MODULE.FRONT_RIGHT))
            /(4);
  }

  /**
   * Returns the velocity of a specific wheel module in feet per second
   * 
   * @param module The wheel module to return the angular velocity of
   * @return The velocity of the wheel module in feet per second
   */
  public double getVelocity(MODULE module){
    return this.module.get(module).getVelocity();
  }

  /**
   * Returns the average velocity of the wheel modules in feet per second
   * 
   * @return The average velocity of the wheel modules in feet per second
   */
  public double getVelocity(){
    return (getVelocity(MODULE.BACK_LEFT) +
            getVelocity(MODULE.BACK_RIGHT) +
            getVelocity(MODULE.FRONT_LEFT) +
            getVelocity(MODULE.FRONT_RIGHT))
            /(4);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
      new Rotation2d(getHeading()),
      frontLeft.getState(),
      backLeft.getState(),
      frontRight.getState(),
      backRight.getState());

    // Print all debug values
    if(DRIVE_TRAIN_CONSTANTS.DEBUG){
      SmartDashboard.putNumber("Gyro Heading", getHeading());
      SmartDashboard.putNumber("Gyro Turn Rate", getTurnRate());

      SmartDashboard.putNumber("Total Turning Current Draw", getWheelTurnCurrent());
      SmartDashboard.putNumber("Front Left Turning Current Draw", getWheelTurnCurrent(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Turning Current Draw", getWheelTurnCurrent(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Turning Current Draw", getWheelTurnCurrent(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Turning Current Draw", getWheelTurnCurrent(MODULE.BACK_RIGHT));

      SmartDashboard.putNumber("Average Wheel Heading", getWheelHeading());
      SmartDashboard.putNumber("Front Left Wheel Heading", getWheelHeading(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Wheel Heading", getWheelHeading(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Wheel Heading", getWheelHeading(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Wheel Heading", getWheelHeading(MODULE.BACK_RIGHT));

      SmartDashboard.putNumber("Average Wheel Turn Rate", getWheelTurnRate());
      SmartDashboard.putNumber("Front Left Wheel Turn Rate", getWheelTurnRate(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Wheel Turn Rate", getWheelTurnRate(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Wheel Turn Rate", getWheelTurnRate(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Wheel Turn Rate", getWheelTurnRate(MODULE.BACK_RIGHT));

      SmartDashboard.putNumber("Total Drive Current Draw", getDriveCurrent());
      SmartDashboard.putNumber("Front Left Drive Current Draw", getDriveCurrent(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Drive Current Draw", getDriveCurrent(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Drive Current Draw", getDriveCurrent(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Drive Current Draw", getDriveCurrent(MODULE.BACK_RIGHT));

      SmartDashboard.putNumber("Average Position", getPosition());
      SmartDashboard.putNumber("Front Left Position", getPosition(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Position", getPosition(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Position", getPosition(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Position", getPosition(MODULE.BACK_RIGHT));

      SmartDashboard.putNumber("Average Velocity", getVelocity());
      SmartDashboard.putNumber("Front Left Velocity", getVelocity(MODULE.FRONT_LEFT));
      SmartDashboard.putNumber("Front Right Velocity", getVelocity(MODULE.FRONT_RIGHT));
      SmartDashboard.putNumber("Back Left Velocity", getVelocity(MODULE.BACK_LEFT));
      SmartDashboard.putNumber("Back Right Velocity", getVelocity(MODULE.BACK_RIGHT));
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DRIVE_TRAIN_CONSTANTS.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(
        swerveModuleStates, 3);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, 3);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}