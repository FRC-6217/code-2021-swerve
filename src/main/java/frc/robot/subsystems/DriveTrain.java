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
import frc.robot.Constants.SWERVE_MODULE_CONSTANTS;
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
    // Assign modules to map
    module.put(MODULE.FRONT_LEFT, frontLeft);
    module.put(MODULE.FRONT_RIGHT, frontRight);
    module.put(MODULE.BACK_LEFT, backLeft);
    module.put(MODULE.BACK_RIGHT, backRight);

    // Put tuning variables on Smartdashboard if tuning enabled
    if(DRIVE_TRAIN_CONSTANTS.ENABLE_TUNING){
      SmartDashboard.putNumber("Turning P", SWERVE_MODULE_CONSTANTS.TURNING_P);
      SmartDashboard.putNumber("Turning I", SWERVE_MODULE_CONSTANTS.TURNING_I);
      SmartDashboard.putNumber("Turning D", SWERVE_MODULE_CONSTANTS.TURNING_D);
      SmartDashboard.putNumber("Turning Max Vel", SWERVE_MODULE_CONSTANTS.MAX_ANGULAR_SPEED_RADIANS);
      SmartDashboard.putNumber("Turning Max Accel", SWERVE_MODULE_CONSTANTS.MAX_ANGULAR_ACCEL_RADIANS);

      SmartDashboard.putBoolean("Enable Drive PID", SWERVE_MODULE_CONSTANTS.ENABLE_DRIVE_PID);
      SmartDashboard.putNumber("Drive P", SWERVE_MODULE_CONSTANTS.DRIVE_P);
      SmartDashboard.putNumber("Drive I", SWERVE_MODULE_CONSTANTS.DRIVE_I);
      SmartDashboard.putNumber("Drive D", SWERVE_MODULE_CONSTANTS.DRIVE_D);
      SmartDashboard.putNumber("Drive FF", SWERVE_MODULE_CONSTANTS.DRIVE_FF);
      SmartDashboard.putNumber("Drive IZone", SWERVE_MODULE_CONSTANTS.DRIVE_IZONE);
      SmartDashboard.putNumber("Drive Min Out", SWERVE_MODULE_CONSTANTS.DRIVE_MIN_OUT);
      SmartDashboard.putNumber("Drive Max Out", SWERVE_MODULE_CONSTANTS.DRIVE_MAX_OUT);
    }
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
    return (getWheelTurnCurrent(MODULE.FRONT_LEFT) +
            getWheelTurnCurrent(MODULE.FRONT_RIGHT) +
            getWheelTurnCurrent(MODULE.BACK_LEFT) +
            getWheelTurnCurrent(MODULE.BACK_RIGHT));
  }

  /**
   * Returns the heading of a specific wheel module from -180 to 180 deg
   * 
   * @param module The wheel module to return the heading of
   * @return The heading of the wheel module from -180 to 180 deg
   */
  public double getWheelHeading(MODULE module){
    return Math.toDegrees(this.module.get(module).getAngle());
  }

  /**
   * Returns the average heading of the wheel modules from -180 to 180 deg
   * 
   * @return The average heading of the wheel modules from -180 to 180 deg
   */
  public double getWheelHeading(){
    return (getWheelHeading(MODULE.FRONT_LEFT) +
            getWheelHeading(MODULE.FRONT_RIGHT) +
            getWheelHeading(MODULE.BACK_LEFT) +
            getWheelHeading(MODULE.BACK_RIGHT))
            /(4);
  }

  /**
   * Returns the angular velocity of a specific wheel module in degrees per second
   * 
   * @param module The wheel module to return the angular velocity of
   * @return The angular velocity of the wheel module in degrees per second
   */
  public double getWheelTurnRate(MODULE module){
    return Math.toDegrees(this.module.get(module).getAngularVelocity());
  }

  /**
   * Returns the average angular velocity of the wheel modules in degrees per second
   * 
   * @return The average angular velocity of the wheel modules in degrees per second
   */
  public double getWheelTurnRate(){
    return (getWheelTurnRate(MODULE.FRONT_LEFT) +
            getWheelTurnRate(MODULE.FRONT_RIGHT) +
            getWheelTurnRate(MODULE.BACK_LEFT) +
            getWheelTurnRate(MODULE.BACK_RIGHT))
            /(4);
  }

  /** Resets a specific wheel module's drive encoder to read a position of 0. */
  public void resetPosition(MODULE module){
    this.module.get(module).resetPosition();
  }

  /** Resets the drive encoders to read a position of 0. */
  public void resetPosition(){
    resetPosition(MODULE.FRONT_LEFT);
    resetPosition(MODULE.FRONT_RIGHT);
    resetPosition(MODULE.BACK_LEFT);
    resetPosition(MODULE.BACK_RIGHT);
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
    return (getDriveCurrent(MODULE.FRONT_LEFT) +
            getDriveCurrent(MODULE.FRONT_RIGHT) +
            getDriveCurrent(MODULE.BACK_LEFT) +
            getDriveCurrent(MODULE.BACK_RIGHT));
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
    return (getPosition(MODULE.FRONT_LEFT) +
            getPosition(MODULE.FRONT_RIGHT) +
            getPosition(MODULE.BACK_LEFT) +
            getPosition(MODULE.BACK_RIGHT))
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
    return (getVelocity(MODULE.FRONT_LEFT) +
            getVelocity(MODULE.FRONT_RIGHT) +
            getVelocity(MODULE.BACK_LEFT) +
            getVelocity(MODULE.BACK_RIGHT))
            /(4);
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

  /**
   * Sets the P constant of the turning PID controllers
   * 
   * @param p The desired P constant of the turning PID controllers
   */
  public void setTurningP(double p){
    this.module.get(MODULE.FRONT_LEFT).setTurningP(p);
    this.module.get(MODULE.FRONT_RIGHT).setTurningP(p);
    this.module.get(MODULE.BACK_LEFT).setTurningP(p);
    this.module.get(MODULE.BACK_RIGHT).setTurningP(p);
  }


  /**
   * Sets the I constant of the turning PID controllers
   * 
   * @param i The desired I constant of the turning PID controllers
   */
  public void setTurningI(double i){
    this.module.get(MODULE.FRONT_LEFT).setTurningI(i);
    this.module.get(MODULE.FRONT_RIGHT).setTurningI(i);
    this.module.get(MODULE.BACK_LEFT).setTurningI(i);
    this.module.get(MODULE.BACK_RIGHT).setTurningI(i);
  }

  /**
   * Sets the D constant of the turning PID controllers
   * 
   * @param d The desired D constant of the turning PID controllers
   */
  public void setTurningD(double d){
    this.module.get(MODULE.FRONT_LEFT).setTurningD(d);
    this.module.get(MODULE.FRONT_RIGHT).setTurningD(d);
    this.module.get(MODULE.BACK_LEFT).setTurningD(d);
    this.module.get(MODULE.BACK_RIGHT).setTurningD(d);
  }

  /**
   * Sets the velocity and acceleration constraints of the turning PID controllers
   * 
   * @param maxVelocityRad The desired max angular velocity of the turning PID controllers in radians per second
   * @param maxAccelerationRad The desired max angular acceleration of the turning PID controllers in radians per second squared
   */
  public void setConstraints(double maxVelocityRad, double maxAccelerationRad){
    this.module.get(MODULE.FRONT_LEFT).setConstraints(maxVelocityRad, maxAccelerationRad);
    this.module.get(MODULE.FRONT_RIGHT).setConstraints(maxVelocityRad, maxAccelerationRad);
    this.module.get(MODULE.BACK_LEFT).setConstraints(maxVelocityRad, maxAccelerationRad);
    this.module.get(MODULE.BACK_RIGHT).setConstraints(maxVelocityRad, maxAccelerationRad);
  }

  /**
   * Sets the state of the drive PID controllers to on or off
   * 
   * @param state The desired state of the drive PID controllers
   */
  public void enableDrivePID(boolean state){
    this.module.get(MODULE.FRONT_LEFT).enableDrivePID(state);
    this.module.get(MODULE.FRONT_RIGHT).enableDrivePID(state);
    this.module.get(MODULE.BACK_LEFT).enableDrivePID(state);
    this.module.get(MODULE.BACK_RIGHT).enableDrivePID(state);
  }

  /**
   * Sets the P constant of the drive PID controllers
   * 
   * @param p The desired P constant of the drive PID controllers
   */
  public void setDriveP(double p){
    this.module.get(MODULE.FRONT_LEFT).setDriveP(p);
    this.module.get(MODULE.FRONT_RIGHT).setDriveP(p);
    this.module.get(MODULE.BACK_LEFT).setDriveP(p);
    this.module.get(MODULE.BACK_RIGHT).setDriveP(p);
  }

  /**
   * Sets the I constant of the drive PID controllers
   * 
   * @param i The desired I constant of the drive PID controllers
   */
  public void setDriveI(double i){
    this.module.get(MODULE.FRONT_LEFT).setDriveI(i);
    this.module.get(MODULE.FRONT_RIGHT).setDriveI(i);
    this.module.get(MODULE.BACK_LEFT).setDriveI(i);
    this.module.get(MODULE.BACK_RIGHT).setDriveI(i);
  }

  /**
   * Sets the D constant of the drive PID controllers
   * 
   * @param d The desired D constant of the drive PID controllers
   */
  public void setDriveD(double d){
    this.module.get(MODULE.FRONT_LEFT).setDriveD(d);
    this.module.get(MODULE.FRONT_RIGHT).setDriveD(d);
    this.module.get(MODULE.BACK_LEFT).setDriveD(d);
    this.module.get(MODULE.BACK_RIGHT).setDriveD(d);
  }

  /**
   * Sets the feed forward constant of the drive PID controllers
   * 
   * @param feedForward The desired feed forward constant of the drive PID controllers
   */
  public void setDriveFF(double feedForward){
    this.module.get(MODULE.FRONT_LEFT).setDriveFF(feedForward);
    this.module.get(MODULE.FRONT_RIGHT).setDriveFF(feedForward);
    this.module.get(MODULE.BACK_LEFT).setDriveFF(feedForward);
    this.module.get(MODULE.BACK_RIGHT).setDriveFF(feedForward);
  }

  /**
   * Sets the integration zone constant of the drive PID controllers
   * 
   * @param iZone The desired integration zone constant of the drive PID controllers
   */
  public void setDriveIZone(double iZone){
    this.module.get(MODULE.FRONT_LEFT).setDriveIZone(iZone);
    this.module.get(MODULE.FRONT_RIGHT).setDriveIZone(iZone);
    this.module.get(MODULE.BACK_LEFT).setDriveIZone(iZone);
    this.module.get(MODULE.BACK_RIGHT).setDriveIZone(iZone);
  }

  /**
   * Sets the output range of the drive PID controllers
   * 
   * @param min The desired min output of the drive PID controllers
   * @param max The desired max output of the drive PID controllers
   */
  public void setDrivePIDOutputRange(double min, double max){
    this.module.get(MODULE.FRONT_LEFT).setDrivePIDOutputRange(min, max);
    this.module.get(MODULE.FRONT_RIGHT).setDrivePIDOutputRange(min, max);
    this.module.get(MODULE.BACK_LEFT).setDrivePIDOutputRange(min, max);
    this.module.get(MODULE.BACK_RIGHT).setDrivePIDOutputRange(min, max);
  }
    

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
      new Rotation2d(getHeading()),
      frontLeft.getState(),
      backLeft.getState(),
      frontRight.getState(),
      backRight.getState());

    // Print all DriveTrain sensor readings if debug is enabled
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

    //Fetch and update PID values if tuning enabled
    if(DRIVE_TRAIN_CONSTANTS.ENABLE_TUNING){
      setTurningP(SmartDashboard.getNumber("Turning P", SWERVE_MODULE_CONSTANTS.TURNING_P));
      setTurningI(SmartDashboard.getNumber("Turning I", SWERVE_MODULE_CONSTANTS.TURNING_I));
      setTurningD(SmartDashboard.getNumber("Turning D", SWERVE_MODULE_CONSTANTS.TURNING_D));
      setConstraints(SmartDashboard.getNumber("Turning Max Vel", SWERVE_MODULE_CONSTANTS.MAX_ANGULAR_SPEED_RADIANS), 
                     SmartDashboard.getNumber("Turning Max Accel", SWERVE_MODULE_CONSTANTS.MAX_ANGULAR_ACCEL_RADIANS));

      enableDrivePID(SmartDashboard.getBoolean("Enable Drive PID", SWERVE_MODULE_CONSTANTS.ENABLE_DRIVE_PID));
      setDriveP(SmartDashboard.getNumber("Drive P", SWERVE_MODULE_CONSTANTS.DRIVE_P));
      setDriveI(SmartDashboard.getNumber("Drive I", SWERVE_MODULE_CONSTANTS.DRIVE_I));
      setDriveD(SmartDashboard.getNumber("Drive D", SWERVE_MODULE_CONSTANTS.DRIVE_D));
      setDriveFF(SmartDashboard.getNumber("Drive FF", SWERVE_MODULE_CONSTANTS.DRIVE_FF));
      setDriveIZone(SmartDashboard.getNumber("Drive IZone", SWERVE_MODULE_CONSTANTS.DRIVE_IZONE));
      setDrivePIDOutputRange(SmartDashboard.getNumber("Drive Min Out", SWERVE_MODULE_CONSTANTS.DRIVE_MIN_OUT),
                             SmartDashboard.getNumber("Drive Max Out", SWERVE_MODULE_CONSTANTS.DRIVE_MAX_OUT));
    }
  }
}