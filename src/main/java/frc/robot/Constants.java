package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    static class ArmConstants{
        static final double ARM_GEARING = (65/12)*50;
        static final double EXTENDER_GEARING = 1/5;
        static final double ARM_LENGTH = Units.feetToMeters(4);
        static final double MAX_ROTATION = 45; // in rotations
        static final double MIN_ROTATION = 0; // in rotations
        static final double MIN_ROT = Units.degreesToRadians(360-55);
        static final double MAX_ROT = Units.degreesToRadians(180+42);
    }
    static class PenumaticConstants {
        static final double MAX_PRESSURE = 60;//PSI
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