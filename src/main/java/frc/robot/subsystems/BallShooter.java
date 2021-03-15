package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BALL_SHOOTER_CONSTANTS;

/** BallShooter */
public class BallShooter extends SubsystemBase {

  // Create objects for motor controllers
 private CANSparkMax top;
  private CANSparkMax bottom;

  // Create objects for integrated encoders
  private CANEncoder topEnc;
  private CANEncoder bottomEnc;

  // Create object for pdp
  private PowerDistributionPanel pdp;

  //Create objects for PID controllers
  private CANPIDController topPID;
  private CANPIDController bottomPID;

  // Create local variable to control state of ball shooter PID controller
  private boolean PIDEnabled = BALL_SHOOTER_CONSTANTS.ENABLE_PID;

  /** Creates a new BallShooter. */
  public BallShooter() {
    // Instantiate motor controller objects
    top = new CANSparkMax(BALL_SHOOTER_CONSTANTS.MOTOR_CONTROLLER_ID_TOP, MotorType.kBrushless);
    bottom = new CANSparkMax(BALL_SHOOTER_CONSTANTS.MOTOR_CONTROLLER_ID_BOTTOM, MotorType.kBrushless);

    // Fetch integrated encoder objects from motor controllers
    topEnc = top.getEncoder();
    bottomEnc = bottom.getEncoder();

    // Instantiate pdp
    pdp = new PowerDistributionPanel();

    // Fetch PID controllers from motor controllers
    topPID = top.getPIDController();
    bottomPID = bottom.getPIDController();

    // Restore defaults to clear any configs
    top.restoreFactoryDefaults();
    bottom.restoreFactoryDefaults();

    // Set idle mode to brake mode -- prevents coasting
    top.setIdleMode(IdleMode.kBrake);
    bottom.setIdleMode(IdleMode.kBrake);

    // Invert motor directions as needed
    top.setInverted(BALL_SHOOTER_CONSTANTS.IS_NEGATED_TOP);
    bottom.setInverted(BALL_SHOOTER_CONSTANTS.IS_NEGATED_BOTTOM);

    // Set PID constants from CONSTANTS file
    topPID.setP(BALL_SHOOTER_CONSTANTS.P);
    topPID.setI(BALL_SHOOTER_CONSTANTS.I);
    topPID.setD(BALL_SHOOTER_CONSTANTS.D);
    topPID.setFF(BALL_SHOOTER_CONSTANTS.FF);
    topPID.setIZone(BALL_SHOOTER_CONSTANTS.IZONE);
    topPID.setOutputRange(BALL_SHOOTER_CONSTANTS.MIN_OUT, BALL_SHOOTER_CONSTANTS.MAX_OUT);

    bottomPID.setP(BALL_SHOOTER_CONSTANTS.P);
    bottomPID.setI(BALL_SHOOTER_CONSTANTS.I);
    bottomPID.setD(BALL_SHOOTER_CONSTANTS.D);
    bottomPID.setFF(BALL_SHOOTER_CONSTANTS.FF);
    bottomPID.setIZone(BALL_SHOOTER_CONSTANTS.IZONE);
    bottomPID.setOutputRange(BALL_SHOOTER_CONSTANTS.MIN_OUT, BALL_SHOOTER_CONSTANTS.MAX_OUT);

    // Put tuning variables on Smartdashboard if tuning enabled
    if(BALL_SHOOTER_CONSTANTS.ENABLE_TUNING){
      SmartDashboard.putBoolean("Enable Shooter PID", BALL_SHOOTER_CONSTANTS.ENABLE_PID);
      SmartDashboard.putNumber("Shooter P", BALL_SHOOTER_CONSTANTS.P);
      SmartDashboard.putNumber("Shooter I", BALL_SHOOTER_CONSTANTS.I);
      SmartDashboard.putNumber("Shooter D", BALL_SHOOTER_CONSTANTS.D);
      SmartDashboard.putNumber("Shooter FF", BALL_SHOOTER_CONSTANTS.FF);
      SmartDashboard.putNumber("Shooter IZone", BALL_SHOOTER_CONSTANTS.IZONE);
      SmartDashboard.putNumber("Shooter Min Out", BALL_SHOOTER_CONSTANTS.MIN_OUT);
      SmartDashboard.putNumber("Shooter Max Out", BALL_SHOOTER_CONSTANTS.MAX_OUT);
    }
  }

  /* Encoder and current handling */
  
  /** Sets the position of the top encoder to 0 */
  public void resetTopEncoder(){
    topEnc.setPosition(0);
  }

  /** Sets the position of the bottom encoder to 0 */
  public void resetBottomEncoder(){
    bottomEnc.setPosition(0);
  }

  /** Sets the position of the top and bottom encoders to 0 */
  public void resetEncoders(){
    resetTopEncoder();
    resetBottomEncoder();
  }

  /**
   * Returns the current draw of the top motor
   * 
   * @return The current draw of the top motor
   */
  public double getTopCurrent(){
    return pdp.getCurrent(BALL_SHOOTER_CONSTANTS.CURRENT_CHANNEL_TOP);
  }

  /**
   * Returns the current draw of the bottom motor in amps
   * 
   * @return The current draw of the bottom motor in amps
   */
  public double getBottomCurrent(){
    return pdp.getCurrent(BALL_SHOOTER_CONSTANTS.CURRENT_CHANNEL_BOTTOM);
  }

  /**
   * Returns the total current draw of both the top and bottom motors
   * 
   * @return The total current draw of both the top and bottom motors
   */
  public double getCurrent(){
    return getTopCurrent() + getBottomCurrent();
  }

  /**
   * Returns the angular position of the top motor in rotations
   *  
   * @return The angular position of the top motor in rotations
   */
  public double getTopRotations(){
    return topEnc.getPosition();
  }

  /**
   * Returns the angular position of the bottom motor in rotations
   *  
   * @return The angular position of the bottom motor in rotations
   */
  public double getBottomRotations(){
    return bottomEnc.getPosition();
  }

  /**
   * Returns the angular velocity of the top motor in RPM
   *  
   * @return The angular velocity of the top motor in RPM
   */
  public double getTopRPM(){
    return topEnc.getVelocity();
  }

  /**
   * Returns the angular velocity of the bottom motor in RPM
   *  
   * @return The angular velocity of the bottom motor in RPM
   */
  public double getBottomRPM(){
    return bottomEnc.getVelocity();
  }

  /* Basic Motor control handling */

  /**
   * Activates the ball shooter with given RPMs
   * 
   * @param topRPM The desired RPM of the top shooter motor
   * @param bottomRPM The desired RPM of the bottom shooter motor
   */
  public void on(double topRPM, double bottomRPM){
    if(PIDEnabled){
      topPID.setReference(topRPM, ControlType.kVelocity);
      bottomPID.setReference(bottomRPM, ControlType.kVelocity);
    }
    else{
      top.set(topRPM / 5000);
      bottom.set(bottomRPM / 5000);
    }
  }

  /** Deactivates the ball shooter */
  public void off(){
    top.set(0);
    bottom.set(0);
  }

  /* PID control */

  /**
   * Sets the state of the ball shooter PID controller to on or off
   * 
   * @param state The desired state of the ball shooter  PID controller
   */
  public void enablePID(boolean state){
    PIDEnabled = state;
  }

  /**
   * Sets the P constant of the PID controller
   * 
   * @param p The desired P constant of the PID controller
   */
  public void setP(double p){
    topPID.setP(p);
    bottomPID.setP(p);
  }

  /**
   * Sets the I constant of the PID controller
   * 
   * @param i The desired I constant of the PID controller
   */
  public void setI(double i){
    topPID.setI(i);
    bottomPID.setI(i);
  }

  /**
   * Sets the D constant of the PID controller
   * 
   * @param d The desired D constant of the PID controller
   */
  public void setD(double d){
    topPID.setD(d);
    bottomPID.setD(d);
  }

  /**
   * Sets the feed forward constant of the PID controller
   * 
   * @param feedForward The desired feed forward constant of the PID controller
   */
  public void setFF(double feedForward){
    topPID.setFF(feedForward);
    bottomPID.setFF(feedForward);
  }

  /**
   * Sets the integration zone constant of the PID controller
   * 
   * @param iZone The desired integration zone constant of the PID controller
   */
  public void setIZone(double iZone){
    topPID.setIZone(iZone);
    bottomPID.setIZone(iZone);
  }

  /**
   * Sets the output range of the PID controller
   * 
   * @param min The desired min output of the PID controller
   * @param max The desired max output of the PID controller
   */
  public void setPIDOutputRange(double min, double max){
    topPID.setOutputRange(min, max);
    bottomPID.setOutputRange(min, max);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all BallShooter sensor readings if debug is enabled
    if(BALL_SHOOTER_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      SmartDashboard.putNumber("Top Current Draw", getTopCurrent());
      SmartDashboard.putNumber("Bottom Current Draw", getBottomCurrent());
      SmartDashboard.putNumber("Shooter Total Current Draw", getCurrent());

      SmartDashboard.putNumber("Top Rotations", getTopRotations());
      SmartDashboard.putNumber("Bottom Rotations", getBottomRotations());

      SmartDashboard.putNumber("Top RPM", getTopRPM());
      SmartDashboard.putNumber("Bottom RPM", getBottomRPM());
    }

    //Fetch and update PID values if tuning enabled
    if(BALL_SHOOTER_CONSTANTS.ENABLE_TUNING){
      enablePID(SmartDashboard.getBoolean("Enable Shooter PID", BALL_SHOOTER_CONSTANTS.ENABLE_PID));
      setP(SmartDashboard.getNumber("Shooter P", BALL_SHOOTER_CONSTANTS.P));
      setI(SmartDashboard.getNumber("Shooter I", BALL_SHOOTER_CONSTANTS.I));
      setD(SmartDashboard.getNumber("Shooter D", BALL_SHOOTER_CONSTANTS.D));
      setFF(SmartDashboard.getNumber("Shooter FF", BALL_SHOOTER_CONSTANTS.FF));
      setIZone(SmartDashboard.getNumber("Shooter IZone", BALL_SHOOTER_CONSTANTS.IZONE));
      setPIDOutputRange(SmartDashboard.getNumber("Shooter Min Out", BALL_SHOOTER_CONSTANTS.MIN_OUT),
                        SmartDashboard.getNumber("Shooter Max Out", BALL_SHOOTER_CONSTANTS.MAX_OUT));
    }
  }
}