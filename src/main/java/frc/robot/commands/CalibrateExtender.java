package frc.robot.commands;

import java.security.AlgorithmParameterGeneratorSpi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ScoringConstants;

public class CalibrateExtender extends CommandBase {
    public static double STEP_SIZE_PERCENT = 0.1;

    ArmSubsystem arm;

    public CalibrateExtender(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        System.out.printf("Calibrating ArmSubsystem: Current is at %.3f\n", arm.getArmCurrent());
        // arm.rawExtenderChange(-STEP_SIZE_PERCENT);
    }

    @Override
    public boolean isFinished() {
        double current = arm.getExtenderCurrent();
        // if(current >= Constants.CurrentLimits.EXTENDER_LIMIT){
        //     return true;
        // }
        return false;
    }
}
