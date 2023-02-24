package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// TODO: Migrate this to PIDCommand()
public class TurnToAngle extends PIDCommand {
    DriveSubsystem drive;
    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
            () -> drive.getPoseWithVisionMeasurements().getRotation().getDegrees(),
            targetAngleDegrees,
            output -> {
                System.out.println("Current angle: " + drive.getHeading() + " deg; target angle is " + targetAngleDegrees + " deg; turn is " + output);
                drive.differentialDrive(0, output);
            },
            drive);

        this.drive = drive;

        getController().enableContinuousInput(-180, 180);

        getController()
            .setTolerance(DriveConstants.kTurnAngleToleranceDegrees, 0.0001);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TurnToAngle: At " 
            + this.m_measurement.getAsDouble() 
            + " deg, setpoint is " 
            + this.m_setpoint.getAsDouble() 
            + " deg, turn is "
            + getController().calculate(m_measurement.getAsDouble())
            + ", error is " 
            + Math.abs(m_setpoint.getAsDouble() 
                - m_measurement.getAsDouble()));
    
        drive.differentialDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
