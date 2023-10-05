package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;

public class SetArm extends CommandBase {
    double armPosition;
    ArmSubsystem arm;

    public SetArm(ArmSubsystem arm, double armPosition) {
        this.arm = arm;
        this.armPosition = armPosition;
    }

    @Override
    public void initialize() {
        System.out.printf("SetArm: Setting arm to %.3f\n", armPosition);
        arm.setArmPosition(armPosition);
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(arm.getArmEncoder().getPosition() - armPosition) < 0.1;
        return false;
    }
}
