// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

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
  
    //encoders
    public final static int leftEncoder = 2;
    public final static int rightEncoder = 3;
    
    public final static int positionEncoder = 11;

    //CANID
    public final static int LEFT_LEAD_CAN_ID = 20;
    public final static int RIGHT_LEAD_CAN_ID = 22;

    public final static int LEFT_FOLLOW_CAN_ID = 21;
    public final static int RIGHT_FOLLOW_CAN_ID = 23;

    //motor config
    public final static double CURRENT_LIMIT = 0.0;

    public final static double OPEN_LOOP_RAMP = 0.0;
    public final static double CLOSED_LOOP_RAMP = 0.0;

    public final static double LEFT_INVERTED = -1.0;
    public final static double RIGH_INVERTED = -1.0;

    //driving
    public final static double JOYSTICK_DEADBAND = 0.0;

  }

  public static class ShooterSupplier {

  }

  public static class Shooter {

  }

  public static class Auto {
    //LTV parameters

    //conversions
    public final static double WHEEL_DIAMETER_METERS = 0.1524; 
    public final static double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS*Math.PI; 
    public final static double DRIVE_GEAR_RATIO = 8.46;
    public final static double POSITION_FACTOR = 0.0;
    public final static double VELOCITY_FACTOR = 0.0;

    //kinematics
    public final static double TRACK_WIDTH_METERS = 0.546;

    //PathPlanner
    public final static double MAX_MODULE_SPEED = 3.0; //m/s
    public final static double MAX_ACCELERATION = 2.0; //m/s^2
    public final static double MAX_ANGULAR_VELOCITY = 540.0; //deg/s
    public final static double MAX_ANGULAR_ACCELERATION = 720.0; //deg/s^2

  }

}
