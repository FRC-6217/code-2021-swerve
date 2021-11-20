package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_INTAKE_CONSTANTS;

/** ShooterIntake */
public class ShooterIntake extends SubsystemBase {
  // Create motor controller object
  private VictorSPX motor;

  // Create pdp object
  /*
  private PowerDistributionPanel pdp;
  */
  /** Creates a new ShooterIntake */
  public ShooterIntake() {
    // Instantiate motor controller object
    motor = new VictorSPX(SHOOTER_INTAKE_CONSTANTS.MOTOR_CONTROLLER_ID);

    // Instantiate pdp object
    /*
    pdp = new PowerDistributionPanel();
*/
    // Invert motor direction as needed
    motor.setInverted(SHOOTER_INTAKE_CONSTANTS.IS_NEGATED);
    
  }

  /**
   * Returns the current draw of the motor in amps
   * 
   * @return The current draw of the motor in amps
   */
  /*
  public double getCurrent(){
    return pdp.getCurrent(SHOOTER_INTAKE_CONSTANTS.CURRENT_CHANNEL);
  }
*/
  /** Activates the motor to intake from the chamber */
  public void intake() {
    motor.set(ControlMode.PercentOutput, SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  /** Activates the motor to eject stuck balls into the chamber */
  public void reverse() {
    motor.set(ControlMode.PercentOutput, -SHOOTER_INTAKE_CONSTANTS.SPEED);
  }

  /** Deactivates the motor */
  public void off() {
    motor.set(ControlMode.PercentOutput, 0);
  }
  
  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all ShooterIntake sensor readings if debug is enabled
    if(SHOOTER_INTAKE_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      // SmartDashboard.putNumber("Shooter Intake Current Draw", getCurrent());
    }
  }
}