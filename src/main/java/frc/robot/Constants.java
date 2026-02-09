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

// NO IMPERIAL UNITS
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }



  public static class Climber {

  }

  public static class DriveTrain {
    // distance from center of robot to center of wheel (cm)
    public final static double WHEEL_DISTANCE = 0.0; // cm
    public final static int LEFT_MOTOR_PORT = 0;
    public final static int RIGHT_MOTOR_PORT = 1;

    //encoders
    public final static int leftEncoder = 2;
    public final static int rightEncoder = 3;
    
    public final static int positionEncoder = 11;

  }

  public static class ShooterSupplier {

  }

  public static class Shooter {

  }

}
