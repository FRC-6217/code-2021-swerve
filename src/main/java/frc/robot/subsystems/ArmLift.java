package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ARM_LIFT_CONSTANTS;

/* ArmLift */
public class ArmLift extends SubsystemBase {
  // Create objects for motors
  private CANSparkMax left;
  private CANSparkMax right;

  // Create pdp object for current sensing
  private PowerDistributionPanel pdp;

  // // Create objects for integrated encoders
  // private CANEncoder leftEnc;
  // private CANEncoder rightEnc;

  // Create objects for limit switches
  private CANDigitalInput upLeftLimit;
  private CANDigitalInput upRightLimit;
  private CANDigitalInput downLeftLimit;
  private CANDigitalInput downRightLimit;


  /** Creates a new ArmLift. */
  public ArmLift() {
    // Instantiate motor controller objects
    left = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_LEFT, MotorType.kBrushed);
    right = new CANSparkMax(ARM_LIFT_CONSTANTS.MOTOR_CONTROLLER_ID_RIGHT, MotorType.kBrushed);

    // Instantiate pdp
    pdp = new PowerDistributionPanel();

    // Instantiate integrated encoder objects
    // leftEnc = left.getEncoder();
    // rightEnc = right.getEncoder();

    // Instantiate limit switch objects
    upLeftLimit = left.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    upRightLimit = right.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    downLeftLimit = left.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    downRightLimit = right.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    // Restores factory defult
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    // Motors will brake instead of coast 
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);

    // Invert motors as needed
    left.setInverted(ARM_LIFT_CONSTANTS.IS_NEGATED_LEFT);
    right.setInverted(ARM_LIFT_CONSTANTS.IS_NEGATED_RIGHT);

    // Set conversion factors for encoders
    // leftEnc.setPositionConversionFactor(360); // Rotations to degrees
    // rightEnc.setPositionConversionFactor(360);

    // leftEnc.setVelocityConversionFactor(6); // RPM to degrees per second
    // rightEnc.setVelocityConversionFactor(6);
    
  }

  // /* Encoder and current handling */
  
  // /** Sets the position of the left encoder to 0 */
  // public void resetLeftEncoder(){
  //   leftEnc.setPosition(0);
  // }

  // /** Sets the position of the right encoder to 0 */
  // public void resetRightEncoder(){
  //   rightEnc.setPosition(0);
  // }

  // /** Sets the position of the left and Right encoders to 0 */
  // public void resetEncoders(){
  //   resetLeftEncoder();
  //   resetRightEncoder();
  // }

  // /**
  //  * Returns the current draw of the left arm motor
  //  * 
  //  * @return The current draw of the left arm motor
  //  */
  // public double getLeftCurrent(){
  //   return pdp.getCurrent(ARM_LIFT_CONSTANTS.CURRENT_CHANNEL_LEFT);
  // }

  // /**
  //  * Returns the current draw of the right arm motor
  //  * 
  //  * @return The current draw of the right arm motor
  //  */
  // public double getRightCurrent(){
  //   return pdp.getCurrent(ARM_LIFT_CONSTANTS.CURRENT_CHANNEL_RIGHT);
  // }

  // /**
  //  * Returns the total current draw of both the left and right arm motors
  //  * 
  //  * @return The total current draw of both the left and right arm motors
  //  */
  // public double getCurrent(){
  //   return getLeftCurrent() + getRightCurrent();
  // }

  // /**
  //  * Returns the angular position of the left arm motor in degrees
  //  *  
  //  * @return The angular position of the left arm motor in degrees
  //  */
  // public double getLeftAngle(){
  //   return leftEnc.getPosition();
  // }

  // /**
  //  * Returns the angular position of the right arm motor in degrees
  //  *  
  //  * @return The angular position of the right arm motor in degrees
  //  */
  // public double getRightAngle(){
  //   return rightEnc.getPosition();
  // }

  // /**
  //  * Returns the angular velocity of the left arm motor in degrees per second
  //  *  
  //  * @return The angular velocity of the left arm motor in degrees per second
  //  */
  // public double getLeftAngularVelocity(){
  //   return leftEnc.getVelocity();
  // }

  // /**
  //  * Returns the angular velocity of the right arm motor in degrees per second
  //  *  
  //  * @return The angular velocity of the right arm motor in degrees per second
  //  */
  // public double getRightAngularVelocity(){
  //   return rightEnc.getVelocity();
  // }

  /* Limit switch handling */

  /**
   * Returns the state of the left upper limit switch
   * 
   * @return The state of the left upper limit switch
   */
  public boolean getLeftUpLimit() {
    return upLeftLimit.get();
  }

  /**
   * Returns the state of the right upper limit switch
   * 
   * @return The state of the right upper limit switch
   */
  public boolean getRightUpLimit() {
    return upRightLimit.get();
  }

  /**
   * Returns the state of the left lower limit switch
   * 
   * @return The state of the left lower limit switch
   */
  public boolean getLeftDownLimit() {
    return downLeftLimit.get();
  }

  /**
   * Returns the state of the right lower limit switch
   * 
   * @return The state of the right lower limit switch
   */
  public boolean getRightDownLimit() {
    return downRightLimit.get();
  }

  /* Motor control handling */

  /** Activate the left arm motor in the upwards direction */
  public void upLeftMotor() {
    left.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  /** Activate the right arm motor in the upwards direction */
  public void upRightMotor() {
    right.set(ARM_LIFT_CONSTANTS.SPEED);
  }

  /** Activate the left arm motor in the downwards direction */
  public void downLeftMotor() {
    left.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  /** Activate the right arm motor in the downwards direction */
  public void downRightMotor() {
    right.set(-ARM_LIFT_CONSTANTS.SPEED);
  }

  /** Deactivates the left motor */
  public void offLeftMotor() {
    left.set(0);
  }
  
  /** Deactivates the right motor */
  public void offRightMotor() {
    right.set(0);
  }
  
  /** Activate the left and right arm motors in the upwards direction */
  public void upMotor() {
    upLeftMotor();
    upRightMotor();
  }

  /** Activate the left and right arm motors in the downwards direction */
  public void downMotor() {
    downLeftMotor();
    downRightMotor();
  }
  
  /** Deactivate the left and right arm motors */
  public void offMotor() {
    offLeftMotor();
    offRightMotor();
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all ArmLift sensor readings if debug is enabled
    if(ARM_LIFT_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      // SmartDashboard.putNumber("Left Current Draw", getLeftCurrent());
      // SmartDashboard.putNumber("Right Current Draw", getRightCurrent());
      // SmartDashboard.putNumber("Arm Total Current Draw", getCurrent());

      // SmartDashboard.putNumber("Left Angle", getLeftAngle());
      // SmartDashboard.putNumber("Right Angle", getRightAngle());

      // SmartDashboard.putNumber("Left Angular Velocity", getLeftAngularVelocity());
      // SmartDashboard.putNumber("Right Angular Velocity", getRightAngularVelocity());

      SmartDashboard.putBoolean("Left Up Limit", getLeftUpLimit());
      SmartDashboard.putBoolean("Right Up Limit", getRightUpLimit());

      SmartDashboard.putBoolean("Left Down Limit", getLeftDownLimit());
      SmartDashboard.putBoolean("Right Down Limit", getRightDownLimit());
    }
  }
}