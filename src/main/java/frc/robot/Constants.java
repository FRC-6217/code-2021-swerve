/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Declaration of Button Mapping ports
    public static final int DRIVESTICK_PORT = 0;
    public static final int XBOX_PORT = 1;

    public static final class DRIVE_TRAIN_CONSTANTS{
        public static final int BR_ANGLE_CURRENT_CHANNEL = 0;
        public static final int BL_ANGLE_CURRENT_CHANNEL = 0;
        public static final int FR_ANGLE_CURRENT_CHANNEL = 0;
        public static final int FL_ANGLE_CURRENT_CHANNEL = 0;
        public static final int BR_ANGLE_MOTOR_ID = 53;
        public static final int BL_ANGLE_MOTOR_ID = 45;
        public static final int FR_ANGLE_MOTOR_ID = 46;
        public static final int FL_ANGLE_MOTOR_ID = 41;
        public static final int BR_DRIVE_CURRENT_CHANNEL = 0;
        public static final int BL_DRIVE_CURRENT_CHANNEL = 0;
        public static final int FR_DRIVE_CURRENT_CHANNEL = 0;
        public static final int FL_DRIVE_CURRENT_CHANNEL = 0;
        public static final int BR_DRIVE_MOTOR_ID = 22;
        public static final int BL_DRIVE_MOTOR_ID = 23;
        public static final int FR_DRIVE_MOTOR_ID = 21;
        public static final int FL_DRIVE_MOTOR_ID = 24;

        public static final double MAX_DRIVE_SPEED_MPS = 0.3;

        public static final double LENGTH = (21.5 * 0.0254); //front to back
        public static final double WIDTH = (24.5 * 0.0254); //Left to Right

        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(LENGTH / 2, WIDTH / 2),
                new Translation2d(LENGTH / 2, -WIDTH / 2),
                new Translation2d(-LENGTH / 2, WIDTH / 2),
                new Translation2d(-LENGTH / 2, -WIDTH / 2));

        public static final boolean GYRO_REVERSED = false;

        public static final boolean DEBUG = false;
        public static final boolean ENABLE_TUNING = false;

        public static enum MODULE {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        };
    }
    public class SWERVE_MODULE_CONSTANTS{
        public static final double WHEEL_DIAMETER_FEET = 0.5;

        public static final double MIN_VOLTAGE = 0.015625;
        public static final double MAX_VOLTAGE = 3.25;

        public static final double TURNING_P = 1.0;
        public static final double TURNING_I = 0.0;
        public static final double TURNING_D = 0.0;
        public static final double MAX_ANGULAR_SPEED_RADIANS = 4 * Math.PI;
        public static final double MAX_ANGULAR_ACCEL_RADIANS = 4 * Math.PI;

        public static final boolean ENABLE_DRIVE_PID = false;
        public static final double DRIVE_P = 0.1;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_FF = 0.0;
        public static final double DRIVE_IZONE = 0.0;
        public static final double DRIVE_MIN_OUT = -1.0;
        public static final double DRIVE_MAX_OUT = 1.0;
        public static final double MAX_DRIVE_SPEED_MPS = DRIVE_TRAIN_CONSTANTS.MAX_DRIVE_SPEED_MPS;
    }
    public class SHOOTER_INTAKE_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID = 52;
        public static final double SPEED = 1;
        public static final boolean IS_NEGATED = true;
    }
    public class NOT_SHOOTER_INTAKE_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID = 32;
        public static final double SPEED = .4;
        public static final boolean IS_NEGATED = true;
    }
    public class ARM_LIFT_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID_LEFT = 28;
        public static final int MOTOR_CONTROLLER_ID_RIGHT = 30;
        public static final int CURRENT_CHANNEL_LEFT = 0;
        public static final int CURRENT_CHANNEL_RIGHT = 0;
        public static final double SPEED = .5;
        public static final boolean IS_NEGATED_LEFT = false;
        public static final boolean IS_NEGATED_RIGHT = false;
        public static final boolean DEBUG = false;
    }
    public class BALL_SHOOTER_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID_TOP = 25;
        public static final int MOTOR_CONTROLLER_ID_BOTTOM = 27;
        public static final double SPEED = 1;
        public static final boolean IS_NEGATED_TOP = true;
        public static final boolean IS_NEGATED_BOTTOM = false;
        public static final boolean ENABLE_TUNING = true;
        public static final double KP = 0.0001;
        public static final double KI = 0;
        public static final double KD = 0.001;
        public static final double KIZ = 0;
        public static final double KFF = 0.000182;
        public static final double KMINOUTPUT = -1;
        public static final double KMAXOUTPUT = 1;
      
    }
    public class ALIGN_COMMAND_CONSTANTS{
        public static final double kPZ = 0;
        public static final double kIZ = 0;
        public static final double kDZ = 0;
        public static final double kPY = 0;
        public static final double kIY = 0;
        public static final double kDY = 0;
    }
    public class WINCH_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID = 26;
        public static final double SPEED = 1;
        public static final boolean IS_NEGATED = true;
    }

    public static class COLOR_WHEEL_CONSTANTS{
        public static final int MOTOR_CONTROLLER_ID = 29;
        public static final double SPEED = .25;
        public static final boolean IS_NEGATED = false;
        public static final I2C.Port PORT = Port.kOnboard;
    }
    
    public static class COLORS_CONSTANTS {
        public static final double[] RGB_FRC_BLUE = { 0.143, 0.427, 0.429 };
        public static final double[] RBG_FRC_GREEN = { 0.197, 0.561, 0.240 };
        public static final double[] RBG_FRC_RED = { 0.460, 0.378, 0.161 };
        public static final double[] RGB_FRC_YELLOW = { 0.361, 0.524, 0.113 };
    }

    public class LIME_LIGHT_CONSTANTS{
        public static final double GOAL_HEIGHT = 83.5;
        public static final double LIME_HEIGHT = 18.5;
        public static final double LIME_ANGLE = 19.5;
    }

    public static enum SIDE {
        LEFT,
        RIGHT,
        BOTH
      };

    public static enum STATE {
        UP,
        DOWN,
        FORWARDS,
        REVERSE,
        OFF  
      };
}


/*
PDP (Device ID 30)	Running Application.	PDP	30	1.30	July 14, 2015	3.1	1.1
Victor SPX (Device ID 40)	Running Application.	Victor SPX	40	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 41)	Running Application.	Victor SPX	41	3.1	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 42)	Running Application.	Victor SPX	42	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 43)	Running Application.	Victor SPX	43	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 44)	Running Application.	Victor SPX	44	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 45)	Running Application.	Victor SPX	45	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 46)	Running Application.	Victor SPX	46	4.22	Nov 19, 2017	0.2	1.0
Victor SPX (Device ID 47)	Running Application.	Victor SPX	47	4.22	Nov 19, 2017	0.2	1.0
Victor SPX.51.arm1	Running Application.	Victor SPX	51	2.121	Nov 19, 2017	0.2	1.0
Victor SPX.52.shooter2	Running Application.	Victor SPX	52	2.121	Nov 19, 2017	0.2	1.0
Victor SPX.53.arm2	Running Application.	Victor SPX	53	2.121	Nov 19, 2017	0.2	1.0
VictorSPX.50.Shooter1	Running Application.	Victor SPX	50	2.121	Nov 19, 2017	0.2	1.0
*/