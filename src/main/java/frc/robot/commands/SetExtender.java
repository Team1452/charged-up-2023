package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSubsystem;

public class SetExtender extends CommandBase {
    double extenderPosition;
    ArmSubsystem arm;

    public SetExtender(ArmSubsystem arm, double extenderPosition) {
        this.arm = arm;
        this.extenderPosition = extenderPosition;
    }

    @Override
    public void initialize() {
        arm.setExtenderPosition(extenderPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getExtenderEncoder().getPosition() - extenderPosition) < 1;
    }
}
