package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.DriveSubsystem;
import frc.robot.Robot;

// TODO: Probably a better way of doing this?
// Also need to turn/move at same time for curve
public class MoveToPoseCurved extends CommandBase {
    PIDController turningPid;
    PIDController distancePid;
    Pose2d targetPose;
    DriveSubsystem drive;

    public MoveToPoseCurved(Pose2d pose, DriveSubsystem drive, Robot robotBase) {
        this.targetPose = pose;
        this.drive = drive;

        double turningPidP = robotBase.turningPidP.getDouble(0);
        double turningPidI = robotBase.turningPidI.getDouble(0);
        double turningPidD = robotBase.turningPidD.getDouble(0);

        double distancePidP = robotBase.distancePidP.getDouble(0);
        double distancePidI = robotBase.distancePidI.getDouble(0);
        double distancePidD = robotBase.distancePidD.getDouble(0);
        
        this.turningPid = new PIDController(turningPidP, turningPidI, turningPidD);
        this.turningPid.setTolerance(0.05);
        this.turningPid.enableContinuousInput(-180, 180);

        this.distancePid = new PIDController(distancePidP, distancePidI, distancePidD);
        this.distancePid.setTolerance(0.1);
    
        this.drive = drive;
    }

    @Override
    public void execute() {
        System.out.println("moveToPoseCurved execute()");
        if (false) { // distancePid.atSetpoint()) {
            System.out.println("At setpoint, stopping robot");
            drive.differentialDrive(0, 0);
        } else {
            Pose2d currentPose = drive.getPose(); 
            double facingAngle = Math.toDegrees(
                Math.atan2(
                    targetPose.getY() - currentPose.getY(),
                    targetPose.getX() - currentPose.getX()));
            double turn = turningPid.calculate(
                currentPose.getRotation().getDegrees(),
                facingAngle
            );

            double distance = Math.hypot(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
            );
            double speed = -distancePid.calculate(distance, 0);

            System.out.printf("not at setpoint, moving with speed: %.4f; turn: %.4f", speed, turn);
            drive.differentialDrive(speed, turn);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.differentialDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return distancePid.atSetpoint() && turningPid.atSetpoint();
    }
}
