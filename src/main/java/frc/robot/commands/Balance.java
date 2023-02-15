package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.DriveSubsystem;

// TODO: Migrate this to PIDCommand()
public class Balance extends PIDCommand {
    public Balance(DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD),
            drive::getPitch,
            0,
            output -> drive.differentialDrive(output, 0),
            drive);

        getController()
            .setTolerance(DriveConstants.kBalanceToleranceDegrees); // TODO: Move to constants
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
