package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NOT_SHOOTER_INTAKE_CONSTANTS;
import frc.robot.Constants.SHOOTER_INTAKE_CONSTANTS;

/** NotShooterIntake */
public class NotShooterIntake extends SubsystemBase {
  // Create motor controller object
  private CANSparkMax motor;

  // Create pdp object
  private PowerDistributionPanel pdp;

  // Create integrated encoder object
  private CANEncoder enc;

  /** Create a new NotShooterIntake */
  public NotShooterIntake() {
    // Instantiate motor controller object
    motor = new CANSparkMax(NOT_SHOOTER_INTAKE_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);
      
    // Instantiate pdp object
    pdp = new PowerDistributionPanel();

    // Instantiate integrated encoder
    enc = motor.getEncoder();

    // Restore defaults to clear any configs
    // Set idle mode to brake mode -- prevents coasting
    // Invert motor as needed
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(NOT_SHOOTER_INTAKE_CONSTANTS.IS_NEGATED);
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
  public double getCurrent(){
    return pdp.getCurrent(SHOOTER_INTAKE_CONSTANTS.CURRENT_CHANNEL);
  }

  /**
   * Returns the angular position of the not intake system
   * 
   * @return The angular position of the not intake system
   */
  public double getRotations(){
    return enc.getPosition();
  }

  /**
   * Returns the angular velocity of the not intake system
   * 
   * @return The angular velocity of the not intake system
   */
  public double getRPM(){
    return enc.getVelocity();
  }

  /** Activates the motor to intake balls into the chamber */
  public void suck() {
    motor.set(NOT_SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  /** Activates the motor to eject stuck balls out of the chamber */
  public void spit() {
    motor.set(-NOT_SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  /** Deavtivates the motor */
  public void off() {
    motor.set(0);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all NotShooterIntake sensor readings if debug is enabled
    if(NOT_SHOOTER_INTAKE_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      SmartDashboard.putNumber("Not Shooter Intake Current Draw", getCurrent());
      SmartDashboard.putNumber("Not Shooter Intake Position", getRotations());
      SmartDashboard.putNumber("Not Shooter Intake Velocity", getRPM());
    }
  }
}