package frc.robot.commands;

import java.security.AlgorithmParameterGeneratorSpi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ScoringConstants;

public class CalibrateArm extends CommandBase {
    ArmSubsystem arm;
    public static double STEP_SIZE_DEGREES = 0.1;

    public CalibrateArm(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        System.out.printf("Calibrating ArmSubsystem: Current is at %.3f\n", arm.getArmCurrent());
        arm.rawArmChange(-STEP_SIZE_DEGREES);
    }

    @Override
    public boolean isFinished() {
        double current = arm.getArmCurrent();
        if(current >= Constants.CurrentLimits.ARM_LIMIT){
            return true;
        }
        return false;
    }
}
