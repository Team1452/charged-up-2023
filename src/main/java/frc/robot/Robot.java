// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain drive = new Drivetrain();
  private final PIDController balancer = new PIDController(0.001, 0, 0);

  Robot() {
    super(500/1000);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    var speed = Math.pow(controller.getLeftY(), 3);
    var rot = Math.pow(controller.getRightX(), 3);

    System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    drive.differentialDrive(speed, rot);
  }

  @Override
  public void testInit() {
  }

  PhotonCameraWrapper pcw = new PhotonCameraWrapper();

  RamseteController ramsete = new RamseteController(1, 0.5);
  PIDController turnPid = new PIDController(0.05, 0, 0);
  PIDController distancePid = new PIDController(0.05, 0, 0);

  @Override
  public void testPeriodic() {
    // Should technically be .getPitch(), but Pigeon
    // isn't oriented/calibrated correctly

    // double pitch = drive.getGyro().getRoll();
    // double speed = balancer.calculate(pitch, 0);
    // System.out.println("pitch: " + pitch + "; speed: " + speed);
    // drive.drive(speed, 0);
  
    var targets = pcw.getTargets(); 
    
    // if (!targets.isEmpty()) {
    //   System.out.println("Detected " + targets.size() + " targets:");
    //   for (var target : targets) {
    //     System.out.println(target.toString());
    //   }
    // }

    Optional<EstimatedRobotPose> estimatedPose = drive.getPcw().getEstimatedGlobalPose(null);

    if (estimatedPose.isPresent()) {
      Pose2d pose = estimatedPose.get().estimatedPose.toPose2d();

      Pose2d targetPose = new Pose2d(-10, 0, new Rotation2d(Units.degreesToRadians(180)));

      double xErr = targetPose.getX() - pose.getX();
      double yErr = targetPose.getY() - pose.getY();

      double headingErr = Math.atan2(xErr, yErr);

      double turnSpeed = turnPid.calculate(headingErr, 0);

      System.out.println("At (" + pose.getX() + ", " + pose.getY() + "), target is (" + targetPose.getX()
        + ", " + targetPose.getY() + "). Heading error (angle) is " + headingErr + ", turning " + turnSpeed);

      if (Math.abs(headingErr) > 0.1) {
        System.out.println("Turning with speed " + turnSpeed);
        drive.differentialDrive(0, turnSpeed);
      } else {
        double distance = Math.sqrt(xErr*xErr + yErr*yErr);
        double driveSpeed = distancePid.calculate(distance, 0);
        
        System.out.println("Now closing in on distance, distance is " + distance);
        
        if (distance > 3) {
          System.out.println("Driving with drive speed " + driveSpeed);
          drive.differentialDrive(driveSpeed, 0);
        } else {
          double yawErr = targetPose.getRotation().getRadians()
            - pose.getRotation().getRadians();
          double yawCorrectionSpeed = turnPid.calculate(yawErr, 0);
          System.out.println("Correcting yaw with speed: " + yawCorrectionSpeed);
          drive.differentialDrive(0, yawCorrectionSpeed);
        }
      }




      // ChassisSpeeds speeds = ramsete.calculate(pose.toPose2d(), targetPose, 0, 0);

      // double forward = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
      // double turn = -speeds.omegaRadiansPerSecond;

      // if (Math.abs(forward) > 1 || Math.abs(turn) > 1) {
      //   double maxSpeed = Math.max(Math.abs(forward), Math.abs(turn));
      //   forward /= maxSpeed;
      //   turn /= maxSpeed;
      // }

      // System.out.print("Error is " + targetPose.minus(pose.toPose2d()) + "; ");
      // System.out.println("Driving: " + forward + ", " + turn);

      // drive.differentialDrive(forward, turn);
    }
  }
}
