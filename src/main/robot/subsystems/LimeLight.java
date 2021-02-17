/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.Angle;
import frc.robot.libraries.Distance;
import frc.robot.Constants.LIME_LIGHT_CONSTANTS;

public class LimeLight extends SubsystemBase {

  private Angle angle;
  private Distance distance;

  private NetworkTable table;
	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
  private NetworkTableEntry tv;

  private int numClients = 0;

  public LimeLight(Angle angle, Distance distance) {
    this.angle = angle;
    this.distance = distance;

    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(0);
  	tx = table.getEntry("tx");
	  ty = table.getEntry("ty");
    tv = table.getEntry("tv");
  }

  public void limeRequired() {
    numClients++;
    table.getEntry("pipeline").setNumber(1);
    /*0 	use the LED Mode set in the current pipeline
    1 	force off*/
  }

  public void limeNotRequired() {
    numClients--;
    if (numClients == 0) {
      table.getEntry("pipeline").setNumber(0);
    }
    /* 0 	Vision processor
1 	Driver Camera (Increases exposure, disables vision processing) */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pipeline", table.getEntry("pipeline").getDouble(1000000));

    if(tv.getDouble(0) == 1){
      SmartDashboard.putBoolean("Target Aquired", true);

      angle.setAngle(-ty.getDouble(Double.POSITIVE_INFINITY));

      distance.setDistance((LIME_LIGHT_CONSTANTS.GOAL_HEIGHT - LIME_LIGHT_CONSTANTS.LIME_HEIGHT)/(Math.tan(LIME_LIGHT_CONSTANTS.LIME_ANGLE + tx.getDouble(0))));
    }
    else{
      SmartDashboard.putBoolean("Target Aquired", false);

      angle.setAngle(Double.POSITIVE_INFINITY);

      distance.setDistance(Double.POSITIVE_INFINITY);
    }
  }
}
