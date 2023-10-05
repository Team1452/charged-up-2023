package frc.robot.commands;

import java.security.AlgorithmParameterGeneratorSpi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ScoringConstants;

public class SetArmAndExtender extends CommandBase {
    double targetArmPosition, targetExtenderPosition;
    ArmSubsystem arm;
    boolean stowing = true;
    int ticks = 0;
    public SetArmAndExtender(ArmSubsystem arm, double armPosition, double extenderPosition) {
        this.arm = arm;
        this.targetArmPosition = armPosition;
        this.targetExtenderPosition = extenderPosition;
    }

    @Override
    public void initialize() {
        // arm.setArmPosition(this.targetArmPosition);
        arm.setExtenderPosition(0);
    }

    @Override
    public void execute() {
        ticks++;
        double armError = Math.abs(arm.getArmPosition() - targetArmPosition);
        double extenderError = Math.abs(arm.getExtenderPosition() - targetExtenderPosition);
        System.out.println("Arm error: " + armError + "; extender error: " + extenderError);

        if (stowing && arm.getExtenderEncoder().getPosition() < 1) {
            stowing = false;
            arm.setExtenderPosition(targetExtenderPosition);
        }
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(arm.getArmEncoder().getPosition() - targetArmPosition) < ScoringConstants.ARM_TOLERANCE_DEGREES
        //     && Math.abs(arm.getExtenderEncoder().getPosition() - targetExtenderPosition) < ScoringConstants.EXTENDER_TOLERANCE_ROTATIONS;
        return ticks > 500;
    }
}
