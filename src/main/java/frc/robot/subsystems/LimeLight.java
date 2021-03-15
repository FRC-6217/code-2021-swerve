package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.libraries.Angle;
import frc.robot.libraries.Distance;
import frc.robot.Constants;
import frc.robot.Constants.LIME_LIGHT_CONSTANTS;

/** LimeLight */
public class LimeLight extends SubsystemBase {

  // Create objects to handle angle and distance values
  private Angle angle;
  private Distance distance;

  // Create Network table objects
  private NetworkTable table;
	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
  private NetworkTableEntry tv;

  // Create counter to track number of limelight users
  private int numClients = 0;

  /**
   * Creates a new LimeLight
   * 
   * @param angle The Angle object to be updated by limelight measuments
   * @param distance The Distance object to be updated by limelight measuments
   */
  public LimeLight(Angle angle, Distance distance) {
    // Pass parameter angle and distance objects to local class objects
    this.angle = angle;
    this.distance = distance;

    // Initialize limelight network table
    table = NetworkTableInstance.getDefault().getTable("limelight");

    // Set limelight pipeline to driver camera mode(disable vision processing)
    table.getEntry("pipeline").setNumber(LIME_LIGHT_CONSTANTS.DRIVER_MODE_PIPELINE);

    // Set local network table value objects to entrys of limelight table object
  	tx = table.getEntry("tx");
	  ty = table.getEntry("ty");
    tv = table.getEntry("tv");
  }

  /** Activates vision processing and reserves camera usage */
  public void limeRequired() {
    //Reserve limelight usage
    numClients++;

    // Set limielight to vision processing mode
    table.getEntry("pipeline").setNumber(LIME_LIGHT_CONSTANTS.VISION_PROCESSING_PIPELINE);
  }

  /** Unreserves camera usage and sets camera to driver mode if no more camera reservations */
  public void limeNotRequired() {
    //unreserve limelight usage
    numClients--;

    // Sets camera to driver mode if no more camera reservations
    if (numClients == 0) {
      table.getEntry("pipeline").setNumber(LIME_LIGHT_CONSTANTS.DRIVER_MODE_PIPELINE);
    }
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    // Determines if limelight sees target and calculates angle and distance values if it does
    if(tv.getDouble(0) == 1){
      angle.setAngle(-ty.getDouble(Double.POSITIVE_INFINITY));
      distance.setDistance((LIME_LIGHT_CONSTANTS.GOAL_HEIGHT - LIME_LIGHT_CONSTANTS.LIME_HEIGHT)/(Math.tan(LIME_LIGHT_CONSTANTS.LIME_ANGLE + tx.getDouble(0))));
    }
    else{
      angle.setAngle(Double.POSITIVE_INFINITY);
      distance.setDistance(Double.POSITIVE_INFINITY);
    }

    // Print all LimeLight sensor readings if debug is enabled
    if(LIME_LIGHT_CONSTANTS.DEBUG || Constants.GLOBAL_DEBUG){
      // Print current operation mode of limelight
      if(table.getEntry("pipeline").getDouble(-1) == LIME_LIGHT_CONSTANTS.DRIVER_MODE_PIPELINE){
        SmartDashboard.putString("Camera Mode", "Driver Mode");
      }
      else if(table.getEntry("pipeline").getDouble(-1) == LIME_LIGHT_CONSTANTS.VISION_PROCESSING_PIPELINE){
        SmartDashboard.putString("Camera Mode", "Vision Processing Mode");
      }

      // Print if target is found
      SmartDashboard.putBoolean("Target Aquired", tv.getDouble(0) == 1);

      // Print current calculated angle and distance
      SmartDashboard.putNumber("Lime Angle", angle.getAngle());
      SmartDashboard.putNumber("Lime Distance", distance.getDistance());
    }
  }
}