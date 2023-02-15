package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.DriveSubsystem;

// TODO: Migrate this to PIDCommand()
public class MoveDistance extends PIDCommand {
    public MoveDistance(double distanceMeters, DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kMoveP, DriveConstants.kMoveI, DriveConstants.kMoveD),
            drive::getPosition,
            drive.getPosition() + distanceMeters,
            output -> drive.differentialDrive(0, output),
            drive);

        getController()
            .setTolerance(0.001); // TODO: Move to constants
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
