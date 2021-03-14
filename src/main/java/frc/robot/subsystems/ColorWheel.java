package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.COLORS_CONSTANTS;
import frc.robot.Constants.COLOR_WHEEL_CONSTANTS;

/** ColorWheel */
public class ColorWheel extends SubsystemBase {
  // Create motor controller object
  private CANSparkMax motor;

  // Create pdp object
  private PowerDistributionPanel pdp;

  // Create integrated encoder object
  private CANEncoder enc;

  // Create color sensor object
  private ColorSensorV3 colorSensor;

  // Create color match objects
  private ColorMatch colorMatch;

  // Create objects to store frc colors 
  private final Color blue;
  private final Color red;
  private final Color yellow;
  private final Color green;
  
  /** Creates a new ColorWheel */
  public ColorWheel() {
    // Instantiate  motor contrroller object
    motor = new CANSparkMax(COLOR_WHEEL_CONSTANTS.MOTOR_CONTROLLER_ID, MotorType.kBrushless);

    // Instantiate pdp object
    pdp = new PowerDistributionPanel();

    // Instantiate integrated encoder
    enc = motor.getEncoder();
    
    // Instantiate color sensor object
    colorSensor = new ColorSensorV3(COLOR_WHEEL_CONSTANTS.PORT);

    // Instantiate color match object
    colorMatch = new ColorMatch();
    
    // Define color objects with specific frc rgb
    blue = ColorMatch.makeColor(COLORS_CONSTANTS.RGB_FRC_BLUE[0], COLORS_CONSTANTS.RGB_FRC_BLUE[1], COLORS_CONSTANTS.RGB_FRC_BLUE[2]);
    red = ColorMatch.makeColor(COLORS_CONSTANTS.RBG_FRC_RED[0], COLORS_CONSTANTS.RBG_FRC_RED[1], COLORS_CONSTANTS.RBG_FRC_RED[2]);
    yellow = ColorMatch.makeColor(COLORS_CONSTANTS.RGB_FRC_YELLOW[0], COLORS_CONSTANTS.RGB_FRC_YELLOW[1], COLORS_CONSTANTS.RGB_FRC_YELLOW[2]);
    green = ColorMatch.makeColor(COLORS_CONSTANTS.RBG_FRC_GREEN[0], COLORS_CONSTANTS.RBG_FRC_GREEN[1], COLORS_CONSTANTS.RBG_FRC_GREEN[2]);

    // Restore defaults to clear any configs
    // Set idle mode to brake mode -- prevents coasting
    // Invert motor as needed
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(COLOR_WHEEL_CONSTANTS.IS_NEGATED);
    
    // Put frc specific color values into color match object
    colorMatch.addColorMatch(blue);
    colorMatch.addColorMatch(red);
    colorMatch.addColorMatch(yellow);
    colorMatch.addColorMatch(green);
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
    return pdp.getCurrent(COLOR_WHEEL_CONSTANTS.CURRENT_CHANNEL);
  }

  /**
   * Returns the angular position of motor in rotations
   * 
   * @return The angular position of motor in rotations
   */
  public double getRotations(){
    return enc.getPosition();
  }

  /**
   * Returns the angular velocity of the motor in RPM
   * 
   * @return The angular velocity of the motor in RPM
   */
  public double getRPM(){
    return enc.getVelocity();
  }

  /**
   * Returns the color seen by the color sensor
   * 
   * @return The color seen by the color sensor
   */
  public Color getColor(){
    return colorSensor.getColor();
  }

  /**
   * Returns the frc color that is closest to the seen color
   * 
   * @return The frc color that is closest to the seen color
   */
  public Color getClosestColor(){
    return colorMatch.matchClosestColor(getColor()).color;
  }

  /**
   * Returns the confidence of the color matcher
   * 
   * @return The confidence of the color matcher
   */
  public double getConfidence(){
    return colorMatch.matchClosestColor(getColor()).confidence;
  }

  /**
   * Returns the proximity of the color matcher
   * 
   * @return The proximity of the color matcher
   */
  public int getProximity(){
    return colorSensor.getProximity();
  }

  /** Sets the motor to spin in the forwards direction */
  public void forwards() {
   motor.set(COLOR_WHEEL_CONSTANTS.SPEED);
  }

  /** Sets the motor to spin in the backwards direction */
  public void backwards() {
    motor.set(-COLOR_WHEEL_CONSTANTS.SPEED);
  }

  /** Deactivates the motor */
  public void off() {
    motor.set(0);
  }
  
  /**
   * Returns the set frc blue color
   * 
   * @return The set frc blue color
   */
  public Color getBlue(){
    return blue;
  }

  /**
   * Returns the set frc green color
   * 
   * @return The set frc green color
   */
  public Color getGreen(){
    return green;
  }

  /**
   * Returns the set frc yellow color
   * 
   * @return The set frc yellow color
   */
  public Color getYellow(){
    return yellow;
  }

  /**
   * Returns the set frc red color
   * 
   * @return The set frc red color
   */
  public Color getRed(){
    return red;
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Print all ColorWheel sensor readings if debug is enabled
    if(COLOR_WHEEL_CONSTANTS.DEBUG){
      SmartDashboard.putNumber("Color Wheel Current Draw", getCurrent());
      SmartDashboard.putNumber("Color Wheel Rotations", getRotations());
      SmartDashboard.putNumber("Color Wheel RPM", getRPM());

      SmartDashboard.putNumber("Red", getColor().red);
      SmartDashboard.putNumber("Green", getColor().green);
      SmartDashboard.putNumber("Blue", getColor().blue);
      SmartDashboard.putString("Detected Color", getClosestColor().toString());
      SmartDashboard.putNumber("Confidence", getConfidence());
      SmartDashboard.putNumber("Proximity", getProximity());
    }
  }
}