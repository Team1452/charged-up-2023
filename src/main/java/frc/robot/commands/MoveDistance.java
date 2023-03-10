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
                // System.out.println("MoveDistance: position is " 
                //     + Units.metersToInches(drive.getPosition()) 
                //     + " inches, target is " + Units.metersToInches(distanceMeters)
                //     + " moving with speed " + output);
                drive.differentialDrive(output, 0);
            },
            drive);
        
        this.drive = drive;
        this.distanceMeters = distanceMeters;

        this.m_controller.setIntegratorRange(-0.05, 0.05);

        getController()
            .setTolerance(0.001); // TODO: Move to constants
    }

    @Override
    public void initialize() {
        double value = drive.getPosition() + distanceMeters;
        this.m_setpoint = () -> value;
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
