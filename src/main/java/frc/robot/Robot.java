// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  // private final Drivetrain drive = new Drivetrain(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_RIGHT);
  private final PIDController balancer = new PIDController(0.001, 0, 0);
  private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);

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
    // var speed = -Math.pow(controller.getLeftY(), 3);
    // var rot = Math.pow(controller.getRightX(), 3);

    // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    arm.set(controller.getLeftY());
    // drive.differentialDrive(speed, rot);

    if (controller.getAButtonPressed()) {
      solenoid.set(Value.kForward);
    } else if (controller.getAButtonReleased()) {
      solenoid.set(Value.kReverse);
    }
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
      
      double target_yaw = Math.atan2(yErr, xErr);
      double current_yaw = pose.getRotation().getRadians();

      double turn = turnPid.calculate(current_yaw, target_yaw);
      
      double distance = Math.sqrt(xErr*xErr + yErr*yErr);

      double speed = -distancePid.calculate(distance, 0);

      // drive.differentialDrive(speed, turn);
    }
  }
}
