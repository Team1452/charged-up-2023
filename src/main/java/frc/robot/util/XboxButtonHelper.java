package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;

public class XboxButtonHelper {
    XboxController controller; 

    private boolean lastAStatus = false,
        lastBStatus = false,
        lastYStatus = false,
        lastXStatus = false;

    public XboxButtonHelper(XboxController controller) {
        this.controller = controller;
    }

    public boolean getAButtonPressed() {
        boolean aStatus = controller.getAButton();
        boolean result = !lastAStatus && aStatus;
        lastAStatus = aStatus;
        return result;
    }

    public boolean getBButtonPressed() {
        boolean bStatus = controller.getBButton();
        boolean result = !lastBStatus && bStatus;
        lastBStatus = bStatus;
        return result;
    }

    public boolean getYButtonPressed() {
        boolean yStatus = controller.getYButton();
        boolean result = !lastYStatus && yStatus;
        lastYStatus = yStatus;
        return result;
    }

    public boolean getXButtonPressed() {
        boolean xStatus = controller.getXButton();
        boolean result = !lastXStatus && xStatus;
        lastXStatus = xStatus;
        return result;
    }
}
