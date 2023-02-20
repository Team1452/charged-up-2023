package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    static class ExtenderConstants {
        static final double EXTENDER_GEARING = 1/5;
        static final double MAX_EXTENDER_ROTATION = 0.0714285671710968; //I hate myself and I hate this number
        //this^ number is for the encoder rotations at max arm length 
        static final double MIN_ARM_LENGTH = Units.feetToMeters(20.5);
        static final double MAX_ARM_LENGTH = Units.feetToMeters(51);
        static final double METERS_PER_ROTATION =  MAX_ARM_LENGTH / MAX_EXTENDER_ROTATION;
        
    }

    static class ArmConstants {
        static final double ARM_GEARING = (65/12)*50;
        static final double ARM_HEIGHT = Units.feetToMeters(4); //height of arm pivot point
        static final double MIN_ROTATION = Units.degreesToRadians(360-55);
        static final double MAX_ROTATION = Units.degreesToRadians(180+42);
    }

    static class PneumaticConstants { 
        static final double MAX_PRESSURE = 60; // PSI
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

    static class VisionConstants {
        static final double ARM_BASE_LENGTH_INCHES = 20.5;
        static final double ARM_EXTENSION_LENGTH_INCHES =  0;//TODO
        static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(16), 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, 16 inches forward of center, half a meter up
        // from center.
        static final String cameraName = "OV5647";
    }
}