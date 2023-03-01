package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class DriveConstants {
        public static final double kMaxSpeed = Units.feetToMeters(10); // 10 ft/s
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kGearRatio = 1/6;
        public static final double kDistancePerPulse = kGearRatio * 2 * Math.PI * kWheelRadius;

        public static final double kMoveP = 0.35;
        public static final double kMoveI = 0.01;
        public static final double kMoveD = 0.10;
        public static final double kMoveToleranceMeters = Units.inchesToMeters(1);

        // public static final double kTurnP = 0.04;
        public static final double kTurnP = 0.1;
        public static final double kTurnI = 0.001;
        public static final double kTurnD = 0.001;
        public static final double kTurnAngleToleranceDegrees = 3;

        public static final double kBalanceP = 0.1;
        public static final double kBalanceI = 0;
        public static final double kBalanceD = 0;
        public static final double kBalanceToleranceDegrees = 1;
    }

    public static class TestbedDriveConstants {
        public static final double kMaxSpeed = Units.feetToMeters(10); // 10 ft/s
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kGearRatio = 1/10.71;
        public static final double kDistancePerPulse = kGearRatio * 2 * Math.PI * kWheelRadius;

        public static final double kMoveP = 0.20;
        public static final double kMoveI = 0.01;
        public static final double kMoveD = 0.01;
        public static final double kMoveToleranceMeters = Units.inchesToMeters(1);

        // public static final double kTurnP = 0.04;
        public static final double kTurnP = 0.01;
        public static final double kTurnI = 0; // .001;
        public static final double kTurnD = 0; // .001;
        public static final double kTurnAngleToleranceDegrees = 1;

        public static final double kBalanceP = 0.1;
        public static final double kBalanceI = 0;
        public static final double kBalanceD = 0;
        public static final double kBalanceToleranceDegrees = 1;
    }

    static class ExtenderConstants {
        static final double EXTENDER_GEARING = 1/5;
        static final double MAX_EXTENDER_POSITION = 15; //I hate myself and I hate this number
        static final double MIN_EXTENDER_POSITION = 0; //I hate myself and I hate this number
        //this^ number is for the encoder rotations at max arm length 
        static final double MIN_ARM_LENGTH = Units.inchesToMeters(20.5);
        static final double MAX_ARM_LENGTH =  Units.inchesToMeters(40) + MIN_ARM_LENGTH;
        static final double METERS_PER_ROTATION = MAX_ARM_LENGTH / MAX_EXTENDER_POSITION;

        static final double MIN_ARM_EXTENSION = 0;
        static final double MAX_ARM_EXTENSION = MAX_ARM_LENGTH - MIN_ARM_LENGTH;

        
    }

    static class ArmConstants {
        static final double ARM_GEARING = (65/12)*50;
        static final double ARM_HEIGHT = Units.feetToMeters(4); //height of arm pivot point
        static final double MIN_ROTATION_RAD = Units.degreesToRadians(360-55);
        static final double MAX_ROTATION_RAD = Units.degreesToRadians(180+42);
        static final double MAX_ROTATION_ROT = 60;
        static final double MIN_ROTATION_ROT = 0;
    }

    static class PneumaticConstants { 
        static final double MAX_PRESSURE = 120; // PSI
        static final double TARGET_PRESSURE = 110; // PSI
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