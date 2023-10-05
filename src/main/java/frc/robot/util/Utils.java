package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class Utils {
    public static boolean atSetpoint(PIDController controller) {
        // For whatever reason .getSetpoint() in PIDController
        // always returns false
        return Math.abs(controller.getPositionError()) < controller.getPositionTolerance()
            && Math.abs(controller.getVelocityError()) < controller.getVelocityTolerance();
    }

    public static double deadzone(double value, double deadzone) {
        return Math.copySign(Math.max(0, Math.abs(value) - deadzone), value) * 1/(1 - deadzone);
    }

    public static double limitMagnitude(double value, double limit) {
        return Math.copySign(Math.min(Math.abs(value), limit), value);
    }
}
