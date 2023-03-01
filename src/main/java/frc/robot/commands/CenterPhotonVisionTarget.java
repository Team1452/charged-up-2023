package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// TODO: Migrate this to PIDCommand()
public class CenterPhotonVisionTarget extends PIDCommand {
    DriveSubsystem drive;
    public CenterPhotonVisionTarget(DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
            () -> drive.getPcw().getTarget().getYaw(),
            0,
            output -> {
                drive.differentialDrive(0, output);
            },
            drive);

        this.drive = drive;

        getController()
            .setTolerance(DriveConstants.kTurnAngleToleranceDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        drive.differentialDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getController().getPositionError()) < getController().getPositionTolerance();
    }
}
