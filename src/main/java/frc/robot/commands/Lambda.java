package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO: Migrate this to PIDCommand()
public class Lambda extends CommandBase {
    Runnable lambda;

    public Lambda(Runnable lambda) {
        this.lambda = lambda;
    }
    
    @Override
    public void initialize() {
        lambda.run();
    }
}
