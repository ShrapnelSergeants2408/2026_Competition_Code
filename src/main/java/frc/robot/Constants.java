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

    public static class ClimberConstants {}

    public static class DriveTrainConstants {

        // // distance from center of robot to center of wheel (cm)
        // public static final double WHEEL_DISTANCE = 0.0; // cm
        public static final int LEFT_MOTOR_PORT = 0;
        public static final int RIGHT_MOTOR_PORT = 1;
    }

    public static class IntakeConstants {}

    public static class ShooterConstants {

        public static final int STALL_LIMIT = 30;
        // CAN IDs
        public static final int SHOOTER_MOTOR_ID = 1;
        public static final int FEEDER_MOTOR_ID = 2;

        // Motor speeds
        public static final double SHOOTER_SPEED = 0.9; // Shoots the ball
        public static final double FEEDER_SPEED = 0.6; // Feeds ball into shooter
        public static final double NOMINAL_VOLTAGE = 12; // i dunno, it was a hardcoded value i moved it
        
         // Current limit for the shooter motor (in amps)
         public static final int SHOOTER_CURRENT_LIMIT = 40; // adjust as needed
         public static final int FEEDER_CURRENT_LIMIT = 30; // adjust as needed
         // PID / feedforward constants
         public static final double TARGET_RPM_10_FEET = 2950.0;
         public static final double SHOOTER_KP = 0.0;       // proportional (starting value)
         public static final double SHOOTER_KI = 0.0;       // integral
         public static final double SHOOTER_KD = 0.0;       // derivative
         public static final double SHOOTER_KV = 0.12;      // feedforward
         public static final double RPM_TOLERANCE = 50.0; // within+50 rpm is ready
        
          // Distance to RPM mapping (distance in feet -> target RPM)
          // You can adjust these RPMs based on shooter testing
         public static final double[] DISTANCES_FEET = {5, 7.5, 12.5, 15, 17.5, 20};
         public static final double[] DISTANCE_RPM_MAP = {2500, 2700, 2950, 3100, 3200, 3300};
       
        }
         public static class SensorConstants {
        public static final int LIGHT_SENSOR_DIO_PORT = 0; // Add your DIO port here

    }
}
