package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double PERIOD_MS = 20;    

    public static class DriveConstants {
        public static double kMaxVoltage = 0.8;
        public static final double kMaxDriveVoltage = 1.0;
        public static final double kMaxAutonVoltage = 0.25;
        public static final double kMaxSpeed = Units.feetToMeters(10); // 10 ft/s
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kGearRatio = 1/12.75;
        public static final double kDistancePerPulse = kGearRatio * 2 * Math.PI * kWheelRadius;

        public static double kVelocityP = 0.001;
        public static double kVelocityI = 0;
        public static double kVelocityD = 0;

        public static double kMoveP = 1.0;
        public static double kMoveI = 0.012;
        public static double kMoveD = 0.0001;
        public static final double kMoveToleranceMeters = Units.inchesToMeters(3);

        // public static final double kTurnP = 0.04;
        public static double kControlTurnP = 0.0033;
    
        public static double kControlTurnI = 0.00003;
        public static double kControlTurnD = 0.0001;

        public static double kTurnP = 0.04;
        public static double kTurnI = 0.0001;
        public static double kTurnD = 0.001;
        public static final double kTurnAngleToleranceDegrees = 1;

        public static double kBalanceP = 0.008;
        public static double kBalanceI = 0.0005;
        public static double kBalanceD = 0;
        public static final double kBalanceToleranceDegrees = 0.1;
    }

    static class ScoringConstants {
        public static double LOW_CONE_NODE_EXTENDER_ROTATIONS = 15.5;
        public static double LOW_CONE_NODE_ARM_ANGLE = 71.12;

        public static double LOW_CUBE_NODE_EXTENDER_ROTATIONS = 10.29;
        public static double LOW_CUBE_NODE_ARM_ANGLE = 77.95;


        public static double HIGH_CONE_NODE_EXTENDER_ROTATIONS = 34.14;
        public static double HIGH_CONE_NODE_ARM_ANGLE = 64.19;

        public static double HIGH_CUBE_NODE_EXTENDER_ROTATIONS = 29.64;
        public static double HIGH_CUBE_NODE_ARM_ANGLE = 71.36;

        /*
        low goal:
            cone:
            15.5 extender rot
            71.12 arm angle

            cube:
            10.2858 extender rot
            77.95 arm angle

        high goal:
            cone:
            34.14 extender rot
            64.19 arm angle 
            
            cube:
            29.64 extender rot
            71.36 arm angle 
        */
    }

    public static class ExtenderConstants {
        public static final double EXTENDER_GEARING = 1/5;

        public static final double MAX_EXTENDER_ROTATIONS = 33.55;
        public static final double MIN_EXTENDER_ROTATIONS = 0;
        public static final double EXTENDER_ROTATION_RANGE = MAX_EXTENDER_ROTATIONS - MIN_EXTENDER_ROTATIONS;

        public static final double MIN_ARM_LENGTH = Units.inchesToMeters(20.5);
        public static final double MAX_ARM_LENGTH = Units.inchesToMeters(60);
        public static final double METERS_PER_ROTATION = Units.inchesToMeters(3.6);

        public static final double MIN_ARM_EXTENSION = 0;
        public static final double MAX_ARM_EXTENSION = MAX_ARM_LENGTH - MIN_ARM_LENGTH;
    }

    public static class ArmConstants {
        public static final double ARM_GEARING = (65/12)*50;
        public static final double ARM_HEIGHT = Units.feetToMeters(4); //height of arm pivot point
        public static final double MIN_ROTATION_RAD = 0;
        public static final double RANGE_RAD = Units.degreesToRadians(26 + 35 + 90);
        public static final double MAX_ROTATION_RAD = MIN_ROTATION_RAD + RANGE_RAD;
        public static final double START_ANGLE = Units.degreesToRadians(61);
        public static final double MAX_ROTATION_ROT = 113.4;
        public static final double MIN_ROTATION_ROT = 0;
        public static final double ARM_ROTATION_RANGE_ROT = MAX_ROTATION_ROT - MIN_ROTATION_ROT;
        public static final double ARM_STRESS_CURRENT = 0;//TODO
    }

    static class PneumaticConstants { 
        static final double MAX_PRESSURE = 200; // PSI
        static final double ANALOG_VCC = 5; // VCC for analog input
    }

    static class DriveTrainConstants {
        static final double kMaxSpeed = Units.feetToMeters(10); // 10 ft/s
        static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        static final double kTrackWidth = 0.381 * 2; // meters
        static final double kWheelRadius = Units.inchesToMeters(7);
        static final int kEncoderResolution = 4096;
        static final double distancePerPulse = 2 * Math.PI * kWheelRadius / (double) kEncoderResolution;
    }

    static class FieldConstants {
        static final double length = Units.feetToMeters(54);
        static final double width = Units.feetToMeters(27);
        //all in meters
        static final double DOUBLE_SUBSTATION_HEIGHT = 0.95;//double substation height from base of platform
        static final double LEVEL_TWO_POLE_HEIGHT =  0.87; //from top of pole
        static final double LEVEL_TWO_PLATFORM_HEIGHT = 0.6;  //from bottom of platform
        static final double LEVEL_THREE_POLE_HEIGHT = 1.17;  //from top of pole
        static final double LEVEL_THREE_PLATFORM_HEIGHT = 0.9;//from bottom of platform
        static final double FLOOR_HEIGHT = 0; //hot take?


    }
    static class ArmExPos{
        //All arm angles here are to be in rotations
        static final double LEVEL_TWO_POLE_ARM_LENGTH = 0; //TODO 
        static final double LEVEL_THREE_POLE_ARM_LENGTH = 0; //TODO
        static final double LEVEL_TWO_PLATFORM_ARM_LENGTH = 0; //TODO
        static final double LEVEL_THREE_PLATFORM_ARM_LENGTH = 0; //TODO

        static final double LEVEL_TWO_POLE_ARM_ANGLE = 0; //TODO
        static final double LEVEL_THREE_POLE_ARM_ANGLE = 0; //TODO
        static final double LEVEL_TWO_PLATFORM_ARM_ANGLE = 0; //TODO
        static final double LEVEL_THREE_PLATFORM_ARM_ANGLE = 0; //TODO
    }

    static class VisionConstants {
        static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(16 /* camera is 16 inches ahead of center */), 0.0, 0.5),
                        // new Translation3d(0, 0, 0),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, 16 inches forward of center, half a meter up
        // from center.
        public static final String cameraName = "OV5647";
    }
}