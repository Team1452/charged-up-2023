// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private Drivetrain drive;
  private int tick;
  private final PIDController balancer = new PIDController(0.001, 0, 0);
  // private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  // private final DoubleSolenoid solenoid = new DoubleSolenoid(
    // PneumaticsModuleType.REVPH, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);


  @Override
  public void robotInit() {
    tick = 0;
    drive = new Drivetrain(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);
  }

  @Override
  public void robotPeriodic() {
    // tick += 1;
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    // Maybe switch to SlewRateLimiter or PID for smoother control?
    var speed = Math.pow(controller.getLeftY(), 3);
    var rot = Math.pow(controller.getRightX(), 3);

    // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);
    drive.differentialDrive(speed, rot);

    // arm.set(controller.getLeftY());

    // if (controller.getAButtonPressed()) {
    //   solenoid.set(Value.kForward);
    // } else if (controller.getAButtonReleased()) {
    //   solenoid.set(Value.kReverse);
    // }
  }

  PhotonCameraWrapper pcw;
  ShuffleboardTab tab = Shuffleboard.getTab("AprilTag Test");

  GenericEntry distanceGraph;
  List<Double> distanceGraphPoints;

  GenericEntry angleErrorGraph;
  List<Double> angleErrorGraphPoints;

  GenericEntry targetPositionLabel;
  GenericEntry currentPositionLabel;

  File aprilTagTestFile;
  FileWriter writer;

  @Override
  public void testInit() {
    pcw = drive.getPcw();
    distanceGraph = tab.add("Distance Graph", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    distanceGraphPoints = new ArrayList<>();

    angleErrorGraph = tab.add("Angle Error Graph", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    angleErrorGraphPoints = new ArrayList<>();

    targetPositionLabel = tab.add("Target Position", 0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    currentPositionLabel = tab.add("Robot Position", 0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    LocalDateTime time = LocalDateTime.now();

    aprilTagTestFile = new File("/home/admin/apriltag_tests/apriltag_test_" + time.getHour() + "_" + time.getMinute() + ".csv");

    try {
      writer = new FileWriter(aprilTagTestFile);
      writer.write("Tick, Current Pose X, Current Pose Y, Current Pose Yaw, Target Pose X, Target Pose Y, Target Pose Yaw");
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void testExit() {
    try {
      writer.close();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }


  RamseteController ramsete = new RamseteController(1, 0.5);
  PIDController turnPid = new PIDController(0.098, 0.002, 0.01);
  PIDController distancePid = new PIDController(0.088, 0.002, 0.01);

  String poseToString(Pose2d pose) {
    return "X: " + pose.getX() + ", Y: " + pose.getY() 
      + ", yaw: " + pose.getRotation().getDegrees();
  }

  boolean targetingIntermediaryTranslation = true;

  @Override
  public void testPeriodic() {
    tick += 1;

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

      // System.out.println("Estimated pose is " + pose);

      Pose2d targetPose = new Pose2d(Units.inchesToMeters(36), 0, new Rotation2d(Units.degreesToRadians(-180)));
      Pose2d relativePose = pose.relativeTo(targetPose);

      double xErr = relativePose.getX();
      double yErr = relativePose.getY();
      
      double target_yaw = Math.atan2(yErr, xErr);

      if (distancePid.atSetpoint()) {
        target_yaw = targetPose.getRotation().getRadians();
      }

      double current_yaw = pose.getRotation().getRadians();


      if (Math.abs(target_yaw - current_yaw) > Math.PI) {
        if (targetingIntermediaryTranslation) {
          targetingIntermediaryTranslation = false;
          turnPid.reset(); // Reset I and D for new yaw setpoint
        }
        current_yaw = target_yaw + 2*Math.PI - Math.abs(relativePose.getRotation().getRadians());
      }

      // System.out.println("Tracking apriltag. xErr: " + xErr + "; yErr: " + yErr + "; angle err: " + (target_yaw - current_yaw));

      // double turn = turnPid.calculate(current_yaw, target_yaw);
      double turn = 0;
      
      // double distance = Math.sqrt(xErr*xErr + yErr*yErr);
      double distance = Math.sqrt(xErr);


      if (tick % 50 == 0) {
        distanceGraphPoints.add(distance);
        distanceGraph.setDoubleArray(distanceGraphPoints.toArray(new Double[distanceGraphPoints.size()]));

        angleErrorGraphPoints.add(target_yaw - current_yaw);
        angleErrorGraph.setDoubleArray(angleErrorGraphPoints.toArray(new Double[distanceGraphPoints.size()]));

        // writer.write("Tick, Current Pose X, Current Pose Y, Current Pose Yaw, Target Pose X, Target Pose Y, Target Pose Yaw");
        try {
          writer.write(tick + ", " + pose.getX() + ", " + pose.getY() + "," + current_yaw
            + "," + targetPose.getX() + "," + targetPose.getY()
            + "," + targetPose.getRotation().getDegrees() + "\n");
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

        System.out.printf("Error: (%f, %f), target yaw: %f, current yaw: %f\n",
          xErr, yErr, target_yaw, current_yaw);
      }

      currentPositionLabel.setString(
        "X: " + pose.getX() + ", Y: " + pose.getY() + ", yaw: " + current_yaw
      );

      targetPositionLabel.setString(
        "X: " + targetPose.getX() + ", Y: " + targetPose.getY() + ", yaw: " + target_yaw
      );

      double speed = distancePid.calculate(distance, 0);

      drive.differentialDrive(speed, 0);
    } else {
      drive.differentialDrive(0, 0);
    }
  }
}
