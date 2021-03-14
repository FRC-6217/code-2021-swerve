package frc.robot.libraries;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.WHEEL_DRIVE_CONSTANTS;

/* SwerveModule */
public class SwerveModule {
  // Create Motor controller objects
  private final VictorSPX turningMotor;
  private final CANSparkMax driveMotor;

  // Create encoder objects
  private final CANAnalog turningEncoder;
  private final CANEncoder driveEncoder;

  // Create PDP object
  private final PowerDistributionPanel pdp = new PowerDistributionPanel();
  private final int turningCurrentID;
  private final int driveCurrentID;

  // Create a TrapezoidProfile PIDController object for the turning motor and initiallize it with constraints to allow for smooth turning
  private final ProfiledPIDController turningPIDController 
    = new ProfiledPIDController(WHEEL_DRIVE_CONSTANTS.TURNING_P, 
                                WHEEL_DRIVE_CONSTANTS.TURNING_I, 
                                WHEEL_DRIVE_CONSTANTS.TURNING_D, 
                                new TrapezoidProfile.Constraints(WHEEL_DRIVE_CONSTANTS.MAX_ANGULAR_SPEED_RADIANS,
                                                                 WHEEL_DRIVE_CONSTANTS.MAX_ANGULAR_ACCEL_RADIANS));

  // Create PID controller object for drive motor
  private final CANPIDController drivePIDController;

  // Create local variables to control 
  private boolean drivePIDEnabled = WHEEL_DRIVE_CONSTANTS.ENABLE_DRIVE_PID;
  private double maxSpeedMPS = WHEEL_DRIVE_CONSTANTS.MAX_DRIVE_SPEED_MPS;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param driveReversed Pass in true to reverse drive direction
   * @param turningReversed Pass in true to reverse turning direction
   */
  public SwerveModule(int turningMotorChannel, int driveMotorChannel, int turningCurrentID, int driveCurrentID, boolean turningReversed, boolean driveReversed) {
    /* Initialization */

    // Initialize motor controller objects with module specific IDs
    // Motor type of drive motor is brushless
    turningMotor = new VictorSPX(turningMotorChannel);
    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

    // Fetch encoder objects from drive motor controller
    // Encoder type of turning encoder is absolute
    turningEncoder = driveMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
    driveEncoder = driveMotor.getEncoder();

    // Pass current channels to local variables
    this.turningCurrentID = turningCurrentID;
    this.driveCurrentID = driveCurrentID;

    // Fetch PID controller object from drive motor
    drivePIDController = driveMotor.getPIDController();


    /* Config */

    // Invert turning motor if object initialized with turningReversed == true
    turningMotor.setInverted(turningReversed);

    // Restore defaults to clear any configs
    // Set idle mode to brake mode -- prevents coasting
    // Invert drive motor if object initialized with driveReversed == true
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(driveReversed);
    driveMotor.setIdleMode(IdleMode.kBrake);

    // Convert the velocity reading of the absolute encoder from revolutions per volt second to radians per volt second
    turningEncoder.setVelocityConversionFactor(2 * Math.PI);

    // Invert drive encoder counting direction if object initialized with driveReversed == true
    // Set position conversion factor of drive encoder to circumference of wheel divided by encoder CPR -- Convert from encoder counts to feet
    // Set velocity conversion factor of drive encoder to circumference of wheel divided by 60 seconds -- Convert from RPM to feet per second
    driveEncoder.setInverted(driveReversed);
    driveEncoder.setPositionConversionFactor((WHEEL_DRIVE_CONSTANTS.WHEEL_DIAMETER_FEET * Math.PI)/(driveEncoder.getCountsPerRevolution()));
    driveEncoder.setVelocityConversionFactor((WHEEL_DRIVE_CONSTANTS.WHEEL_DIAMETER_FEET * Math.PI)/(60));

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Set drive PID parameters to those stored in CONSTANTS
    drivePIDController.setP(WHEEL_DRIVE_CONSTANTS.DRIVE_P);
    drivePIDController.setI(WHEEL_DRIVE_CONSTANTS.DRIVE_I);
    drivePIDController.setD(WHEEL_DRIVE_CONSTANTS.DRIVE_D);
    drivePIDController.setFF(WHEEL_DRIVE_CONSTANTS.DRIVE_FF);
    drivePIDController.setIZone(WHEEL_DRIVE_CONSTANTS.DRIVE_FF);
    drivePIDController.setOutputRange(WHEEL_DRIVE_CONSTANTS.DRIVE_MIN_OUT, WHEEL_DRIVE_CONSTANTS.DRIVE_MAX_OUT);
  }

  private double fit(double value, double minInput, double maxInput, double minOutput, double maxOutput){
    double m = (maxOutput - minOutput)/(maxInput - minInput);
    double b = maxOutput - (m * maxInput);
    return value * m + b;
  }
  
  /**
   * Returns the value of absolute turning encoder on a range from -PI to PI radians 
   * 
   * @return The value of absolute turning encoder on a range from -PI to PI radians
   */
  public double getAngle() {
    return fit(turningEncoder.getPosition(), WHEEL_DRIVE_CONSTANTS.MIN_VOLTAGE, WHEEL_DRIVE_CONSTANTS.MAX_VOLTAGE, -Math.PI, Math.PI);
  }

  /**
   * Returns the current draw of the turning motor
   * 
   * @return The current draw of the turning motor
   */
  public double getTurningCurrent(){
    return pdp.getCurrent(turningCurrentID);
  }


  /**
   * Returns the angular velocity of the turning encoder in radians per volt second
   * 
   * @return The angular velocity of the turning encoder in radians per volt second
   */
  public double getAngularVelocity(){
    return turningEncoder.getVelocity();
  }

  /**
  * Resets the distance traveled by the drive wheel to zero
  */
  public void resetPosition() {
    driveEncoder.setPosition(0);
  }
   
  /**
   * Returns the current draw of the drive motor
   * 
   * @return The current draw of the drive motor
   */
  public double getDriveCurrent(){
    return pdp.getCurrent(driveCurrentID);
  }

  /**
   * Returns the distance traveled by the drive wheel in feet
   * 
   * @return The distance traveled by the drive wheel in feet
   */
  public double getPosition() {
    return driveEncoder.getPosition();
  }

  /**
   * Returns the translational velocity of the drive wheel in feet per second
   * 
   * @return The translational velocity of the drive wheel in feet per second
   */
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = turningPIDController.calculate(getAngle(), state.angle.getRadians());

    // Set turning motor to calculated value
    turningMotor.set(ControlMode.PercentOutput, turnOutput);

    // Set drive motor speed
    if(drivePIDEnabled){
      // Pass in speed request to drive PID controller as RPM
      drivePIDController.setReference(state.speedMetersPerSecond * (60/(WHEEL_DRIVE_CONSTANTS.WHEEL_DIAMETER_FEET * 0.3048 * Math.PI)), ControlType.kVelocity);
    }
    else{
      // Set voltage of drive motor to speed request on range from -1 to 1 
      driveMotor.set(fit(state.speedMetersPerSecond, -maxSpeedMPS, maxSpeedMPS, -1, 1));
    }
  }

  public void setTurningP(double p){
    turningPIDController.setP(p);
  }

  public void setTurningI(double i){
    turningPIDController.setI(i);
  }

  public void setTurningD(double d){
    turningPIDController.setD(d);
  }

  public void setConstraints(double maxVelocityRad, double maxAccelerationRad){
    turningPIDController.setConstraints(new TrapezoidProfile.Constraints(maxVelocityRad, maxAccelerationRad));
  }

  public void enableDrivePID(boolean state){
    drivePIDEnabled = state;
  }

  public void setMaxSpeedMPS(double max){
    maxSpeedMPS = max;
  }

  public void setDriveP(double p){
    drivePIDController.setP(p);
  }

  public void setDriveI(double i){
    drivePIDController.setI(i);
  }

  public void setDriveD(double d){
    drivePIDController.setD(d);
  }

  public void setDriveFF(double feedFoward){
    drivePIDController.setFF(feedFoward);
  }

  public void setDriveIZone(double iZone){
    drivePIDController.setIZone(iZone);
  }

  public void setDrivePIDOutputRange(double min, double max){
    drivePIDController.setOutputRange(min, max);
  }
}