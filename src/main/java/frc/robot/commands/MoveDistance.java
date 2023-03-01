package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.DriveSubsystem;

// TODO: Migrate this to PIDCommand()
public class MoveDistance extends PIDCommand {
    DriveSubsystem drive;
    double distanceMeters;

    public MoveDistance(double distanceMeters, DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kMoveP, DriveConstants.kMoveI, DriveConstants.kMoveD),
            drive::getPosition,
            drive.getPosition() + distanceMeters,
            output -> {
                System.out.println("MoveDistance: position is " 
                    + Units.metersToInches(drive.getPosition()) 
                    + " inches, target is " + Units.metersToInches(distanceMeters));
                drive.differentialDrive(output, 0);
            },
            drive);
        
        this.drive = drive;
        this.distanceMeters = distanceMeters;

        getController()
            .setTolerance(DriveConstants.kMoveToleranceMeters); // TODO: Move to constants
    }

    @Override
    public void initialize() {
        double value = drive.getPosition() + distanceMeters;
        this.m_setpoint = () -> value;
    }

    @Override
    public void end(boolean interrupted) {
        drive.differentialDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Is finished? " + getController().atSetpoint() + "; " + getController().getPositionTolerance() + "; " + getController().getVelocityTolerance() + "; " + getController().getPositionError());
        
        // atSetpoint() would return false even when within tolerance.
        // No idea why? TODO: Figure out why

        // return getController().atSetpoint();
        return Math.abs(getController().getPositionError()) < getController().getPositionTolerance();
    }
}
