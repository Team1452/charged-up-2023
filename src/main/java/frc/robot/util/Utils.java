package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class Utils {
    public static boolean atSetpoint(PIDController controller) {
        // For whatever reason .getSetpoint() in PIDController
        // always returns false
        return Math.abs(controller.getPositionError()) < controller.getPositionTolerance()
            && Math.abs(controller.getVelocityError()) < controller.getVelocityTolerance();
    }
}
