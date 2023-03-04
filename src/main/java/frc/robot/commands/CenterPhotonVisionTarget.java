package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Utils;

// TODO: Migrate this to PIDCommand()
public class CenterPhotonVisionTarget extends PIDCommand {
    DriveSubsystem drive;
    public CenterPhotonVisionTarget(DriveSubsystem drive) {
        super(
            new PIDController(0.001, 0.0002, 0),
            () -> {
                var target = drive.getPcw().getTarget();
                if (target != null) return target.getYaw();
                else return 0;
            },
            0,
            output -> {
                drive.differentialDriveVoltage(0, -output);
            },
            drive);

        this.drive = drive;

        getController()
            .setTolerance(DriveConstants.kTurnAngleToleranceDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        drive.differentialDriveVoltage(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Utils.atSetpoint(m_controller);
    }
}
