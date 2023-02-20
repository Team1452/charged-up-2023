package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// TODO: Migrate this to PIDCommand()
public class TurnToAngle extends PIDCommand {
    DriveSubsystem drive;
    double targetAngleDegrees;
    double lastOutput;

    public void setLastOutput(double lastOutput) {
        this.lastOutput = lastOutput;
    }

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
            drive::getHeading,
            targetAngleDegrees,
            output -> {
                System.out.println("Current angle: " + drive.getHeading() + " deg; target angle is " + targetAngleDegrees + " deg; turn is " + output);
                drive.differentialDrive(0, output);
            },
            drive);

        this.drive = drive;
        this.targetAngleDegrees = targetAngleDegrees;

        getController().enableContinuousInput(-180, 180);

        getController()
            .setTolerance(DriveConstants.kTurnAngleToleranceDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Exiting TurnToAngle, target was " + targetAngleDegrees + ", at " + drive.getHeading() + " deg, final output is " + getController().calculate(drive.getHeading()));
        System.out.println("Killing motors");
        drive.differentialDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
