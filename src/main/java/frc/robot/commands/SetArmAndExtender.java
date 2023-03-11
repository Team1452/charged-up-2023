package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;
import frc.robot.Constants;

public class SetArmAndExtender extends CommandBase {
    double armPosition, extenderPosition;
    ArmSubsystem arm;

    public SetArmAndExtender(ArmSubsystem arm, double armPercent, double extenderPercent) {
        this.arm = arm;
        this.armPosition = armPercent/100 * Constants.ArmConstants.ARM_ROTATION_RANGE_ROT; 
        this.extenderPosition = extenderPercent/100 * Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE;
    }

    @Override
    public void initialize() {
        arm.setArmPosition(this.armPosition);
        arm.setExtenderPosition(this.extenderPosition);
    }

    @Override
    public void execute() {
        double armError = Math.abs(arm.getArmPosition() - armPosition);
        double extenderError = Math.abs(arm.getExtenderPosition() - extenderPosition);
        System.out.println("Arm error: " + armError + "; extender error: " + extenderError);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmEncoder().getPosition() - armPosition) < 1
            && Math.abs(arm.getExtenderEncoder().getPosition() - extenderPosition) < 1;
    }
}
