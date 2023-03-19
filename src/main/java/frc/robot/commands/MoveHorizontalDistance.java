package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Utils;
import frc.robot.Constants;
import frc.robot.DriveSubsystem;

// TODO: Migrate this to PIDCommand()
public class MoveHorizontalDistance extends CommandBase {
    DriveSubsystem drive;
    double distanceMeters;
    PIDController controller;
    double pitchExitThreshold;
    double msElapsedSinceMetPitch = 0;
    double maxVoltage = Constants.DriveConstants.kMaxAutonVoltage;

    /*
     * 39.25" -> 6"
     * 
     */

    public MoveHorizontalDistance(double distanceMeters, DriveSubsystem drive) {
        this.controller = new PIDController(DriveConstants.kMoveP, DriveConstants.kMoveI, DriveConstants.kMoveD);

        this.drive = drive;
        this.distanceMeters = distanceMeters; // Temp empirical scaling factor

        this.pitchExitThreshold = Double.MAX_VALUE;

        this.controller
                .setTolerance(DriveConstants.kMoveToleranceMeters); // TODO: Move to constants
    }

    public MoveHorizontalDistance withMaxVoltage(double maxVoltage) {
        this.maxVoltage = maxVoltage;
        return this;
    }

    public MoveHorizontalDistance withPitchExitThreshold(double degrees) {
        this.pitchExitThreshold = degrees;
        return this;
    }

    public MoveHorizontalDistance withCustomGains(double kP, double kI, double kD) {
        this.controller.setPID(kP, kI, kD);
        return this;
    }

    @Override
    public void initialize() {
        // Scale by empirical constant
        drive.setMaxVoltage(maxVoltage);
        double value = drive.getHorizontalXPosition() + distanceMeters;
        controller.setSetpoint(value);
    }

    @Override
    public void execute() {
        double output = controller.calculate(drive.getHorizontalXPosition());
        System.out.println("MoveDistance: position is "
                + Units.metersToInches(drive.getHorizontalXPosition())
                + " inches, target is " + Units.metersToInches(controller.getSetpoint()) + " output is " + output);

        drive.differentialDrive(output, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setMaxSpeed(Constants.DriveConstants.kMaxVoltage);
        // drive.killMotors();
    }

    @Override
    public boolean isFinished() {
        // atSetpoint() would return false even when within tolerance.
        // No idea why? TODO: Figure out why

        if (Math.abs(drive.getPitch()) > pitchExitThreshold) {
            msElapsedSinceMetPitch += Constants.PERIOD_MS;
        }

        if (msElapsedSinceMetPitch > 500) {
            return true;
        }

        // return getController().atSetpoint();
        boolean isFinished = Utils.atSetpoint(controller);
        if (isFinished) {
            System.out.println("MoveDistance: Finished moving " + distanceMeters + " meters with final error "
                    + controller.getPositionError() + " meters");
        }
        return isFinished;
    }
}
