package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WINCH_CONSTANTS;

/** Winch */
public class Winch extends SubsystemBase {
  // Create motor controller object
  private CANSparkMax motor;

  // Create pdp object
  /*
  private PowerDistributionPanel pdp;
*/
  // Create integrated encoder object
  private CANEncoder enc;

  /** Creates a new Winch */
  public Winch() {
    // Instantiate motor controller object
    motor = new CANSparkMax(WINCH_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    
    // Instantiate pdp object 
    /*
    pdp = new PowerDistributionPanel();
*/
    // Instantiate integrated encoder
    enc = motor.getEncoder();

    // Restore defaults to clear any configs
    // Set idle mode to brake mode -- prevents coasting
    // Invert motor as needed
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(WINCH_CONSTANTS.IS_NEGATED);

    // Set conversion factors of encoder
    enc.setPositionConversionFactor(WINCH_CONSTANTS.POSITION_CONVERSION_FACTOR);
    enc.setVelocityConversionFactor(WINCH_CONSTANTS.VELOCITY_CONVERSION_FACTOR);
  }

  /** Set the encoder position to 0 */
  public void resetEncoder(){
    enc.setPosition(0);
  }

  /**
   * Returns the current draw of the motor in amps
   * 
   * @return The current draw of the motor in amps
   */
  /*
  public double getCurrent(){
    return pdp.getCurrent(WINCH_CONSTANTS.CURRENT_CHANNEL);
  }*/

  /**
   * Returns the position of the winch
   * 
   * @return The position of the winch
   */
  public double getRotations(){
    return enc.getPosition();
  }

  /**
   * Returns the velocity of the winch
   * 
   * @return The velocity of the winch
   */
  public double getRPM(){
    return enc.getVelocity();
  }

  /** Activates the motor to move the climber in the upwards direction */
  public void up() {
    motor.set(WINCH_CONSTANTS.SPEED);
  }

  /** Activates the motor to move the climber in the downwards direction */
  public void down() {
    motor.set(-WINCH_CONSTANTS.SPEED);
  }

  /** Deactivates the motor */
  public void off() {
    motor.set(0);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all Winch sensor readings if debug is enabled
    if(WINCH_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      // SmartDashboard.putNumber("Winch Current Draw", getCurrent());
      SmartDashboard.putNumber("Winch Position", getRotations());
      SmartDashboard.putNumber("Winch Velocity", getRPM());
    }
  }
}