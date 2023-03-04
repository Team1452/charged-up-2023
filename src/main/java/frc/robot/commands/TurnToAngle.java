package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Utils;

// TODO: Migrate this to PIDCommand()
public class TurnToAngle extends PIDCommand {
    DriveSubsystem drive;
    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
        super(
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
            () -> drive.getHeading(),
            targetAngleDegrees,
            output -> {
                System.out.println("Current angle: " + drive.getGyro().getYaw() + " deg; target angle is " + targetAngleDegrees + " deg; turn is " + output);
                drive.differentialDriveVoltage(0, output);
            },
            drive);

        this.drive = drive;

        getController().enableContinuousInput(-180, 180);

        getController()
            .setTolerance(DriveConstants.kTurnAngleToleranceDegrees, 0.01);
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
    
        drive.differentialDriveVoltage(0, 0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Is finished? " + getController().atSetpoint() + "; " + getController().getPositionTolerance() + "; " + getController().getVelocityTolerance() + "; " + getController().getPositionError());
        
        // atSetpoint() would return false even when within tolerance.
        // No idea why? TODO: Figure out why

        // return getController().atSetpoint();
        // return Math.abs(getController().getPositionError()) < getController().getPositionTolerance();
        return Utils.atSetpoint(m_controller);
    }
}
