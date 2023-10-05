package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class DynamicCommand {
    private Command command = null;

    public DynamicCommand() {}

    public boolean isActive() {
        return command != null && command.isScheduled();
    }
    
    public void cancel() {
        if (isActive()) command.cancel();
    }

    public void scheduleNew(Command command) {
        cancel();
        this.command = command;
        this.command.schedule();
    }
}
