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
        public static final double kGearRatio = 1/10.71;
        public static final double kDistancePerPulse = kGearRatio * 2 * Math.PI * kWheelRadius;

        public static final double kMoveP = 0.35;
        public static final double kMoveI = 0.10;
        public static final double kMoveD = 0.05;
        public static final double kMoveToleranceMeters = Units.inchesToMeters(1);

        public static final double kTurnP = 0.04;
        public static final double kTurnI = 0.001;
        public static final double kTurnD = 0;
        public static final double kTurnAngleToleranceDegrees = 5;

        public static final double kBalanceP = 0.1;
        public static final double kBalanceI = 0;
        public static final double kBalanceD = 0;
        public static final double kBalanceToleranceDegrees = 1;
    }

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }

    public static class VisionConstants {
        public static final Transform3d robotToCam =
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