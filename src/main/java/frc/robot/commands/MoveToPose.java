package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.DriveSubsystem;

// TODO: Probably a better way of doing this?
// Also need to turn/move at same time for curve
public class MoveToPose extends CommandBase {
    DriveSubsystem drive;
    Pose2d pose;
    Command command;

    public MoveToPose(Pose2d pose, DriveSubsystem drive) {
        this.pose = pose;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose(); 
        double facingAngle = Math.toDegrees(
            Math.atan2(
                pose.getY() - currentPose.getY(),
                pose.getX() - currentPose.getX()));
        System.out.println("MoveToPose: current angle: " + currentPose.getRotation().getDegrees() + " deg; facing angle: " + facingAngle + "; target angle: " + pose.getRotation().getDegrees());
        command = new TurnToAngle(facingAngle, drive)
            .andThen(
                new MoveDistance(
                    Math.hypot(
                        pose.getX() - drive.getPose().getX(),
                        pose.getY() - drive.getPose().getY()),
                    drive))
            .andThen(new TurnToAngle(pose.getRotation().getDegrees(), drive));
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            if (command != null)
                command.cancel();
        } else {
            Pose2d currentPose = drive.getPose();
            System.out.println("Exiting MoveToPose, target pose was " + pose + ", current pose is " + currentPose);
            System.out.println("Killing motors");
            drive.differentialDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return command == null || command.isFinished();
    }
}
