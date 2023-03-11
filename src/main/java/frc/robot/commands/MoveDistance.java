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
public class MoveDistance extends CommandBase {
    DriveSubsystem drive;
    double distanceMeters;
    PIDController controller;
    double pitchExitThreshold;
    double msElapsedSinceMetPitch = 0;

    /*
     * 39.25" -> 6"
     * 
     */

    public MoveDistance(double distanceMeters, DriveSubsystem drive) {
        this.controller = new PIDController(DriveConstants.kMoveP, DriveConstants.kMoveI, DriveConstants.kMoveD);

        this.drive = drive;
        this.distanceMeters = distanceMeters * 39.25/6; // Temp empirical scaling factor

        this.pitchExitThreshold = Double.MAX_VALUE;

        this.controller
            .setTolerance(DriveConstants.kMoveToleranceMeters); // TODO: Move to constants
    }

    public MoveDistance withPitchExitThreshold(double degrees) {
        this.pitchExitThreshold = degrees;
        return this;
    }

    public MoveDistance withCustomGains(double kP, double kI, double kD) {
        this.controller.setPID(kP, kI, kD);
        return this;
    }

    @Override
    public void initialize() {
        // Scale by empirical constant
        double value = drive.getPosition() + distanceMeters;
        controller.setSetpoint(value);
    }

    @Override
    public void execute() {
        double output = controller.calculate(drive.getPosition());
        System.out.println("MoveDistance: position is " 
            + Units.metersToInches(drive.getPosition()) 
            + " inches, target is " + Units.metersToInches(controller.getSetpoint()));
        
        drive.differentialDrive(output, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.killMotors();
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
            System.out.println("MoveDistance: Finished moving " + distanceMeters + " meters with final error " + controller.getPositionError() + " meters");
        }
        return isFinished;
    }
}
