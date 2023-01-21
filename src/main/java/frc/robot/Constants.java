// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    /** Class for the speed and acceleration limits of the robot. */
    public static class MovementConstraints {
      public static final double dtmaxaccel = 1;
      public static final double dtmaxspeed = 0.6;
      public static final double dtmaxomega = 2.0;
  
      
    }

    public static class JoystickPortIDs {
    public static final int leftJoystickPortID = 0;
    public static final int rightJoystickPortID = 1;
    }

    public static class DrivetrainConstants {
    public static final int leftTopCanID = 1;
    public static final int leftBottomCanID = 2;
    public static final int rightTopCanID = 5;
    public static final int rightBottomCanID = 6;
    public final static double trackWidth = .641 ; // Meters

    }

   /**The drive axis of the joysticks*/
    public static final int JoystickAxis = 1;

    public static class InvertDrivetrain {
    public static final int leftInvert = -1;
    public static final int rightInvert = -1;
    }

    public static class MotorSpeedMultipliers {
    public static final double motorSpeedMultiplier = 0.6;
    public static final double motorSpeedMultiplierLeft = 0.98;
    public static final double motorSpeedMultiplierRight = 1;
    }
    
    public static final double highGear = 0.7;
    public static final double lowGear = 0.5;


}
