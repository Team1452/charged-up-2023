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
import java.util.concurrent.ConcurrentHashMap;

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
import io.javalin.Javalin;
import io.javalin.websocket.WsContext;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private Drivetrain drive;
  private int tick;
  private final PIDController balancer = new PIDController(0.001, 0, 0);
  // private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  // private final DoubleSolenoid solenoid = new DoubleSolenoid(
    // PneumaticsModuleType.REVPH, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);
  private Javalin app;

  RamseteController ramsete = new RamseteController(1, 0.5);
  PIDController turnPid = new PIDController(0.098, 0.002, 0.01);
  PIDController distancePid = new PIDController(0.088, 0.002, 0.01);

  boolean targetingIntermediaryTranslation = true;

  ShuffleboardTab tab = Shuffleboard.getTab("AprilTag Test");

  private final Map<WsContext, String> clients = new ConcurrentHashMap<>();
  private int userNumber = 0;

  @Override
  public void robotInit() {
    tick = 0;
    drive = new Drivetrain(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);
    app = Javalin.create().start(7070);
  
    app.ws("/debug", ws -> {
      ws.onConnect(ctx -> {
        String username = "User" + userNumber++;
        clients.put(ctx, username);
      });

      ws.onClose(ctx -> {
        clients.remove(ctx);
      });
    });
  }

  private void broadcastMessage(String message) {
    for (WsContext ctx : clients.keySet()) {
      ctx.send(message);
    }
  }

  @Override
  public void robotPeriodic() {
    tick += 1;
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


  Pose2d poseAhead;

  @Override
  public void testInit() {
    Pose2d initialPose = drive.getPose();
    double yaw = initialPose.getRotation().getRadians();
    poseAhead = new Pose2d(
      initialPose.getX() + Math.cos(yaw) * Units.inchesToMeters(10),
      initialPose.getY() + Math.sin(yaw) * Units.inchesToMeters(10),
      initialPose.getRotation()
    );
  }

  String poseToString(Pose2d pose) {
    return "X: " + pose.getX() + ", Y: " + pose.getY() 
      + ", yaw: " + pose.getRotation().getDegrees();
  }



  @Override
  public void testPeriodic() {
    // drive.updateOdometry();
    // Pose2d pose = drive.getPose();
    // Pose2d targetPose = poseAhead;

    // // Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(90)));

    // Pose2d relativePose = pose.relativeTo(targetPose);

    // double distance = Math.sqrt(
    //   Math.pow(relativePose.getX(), 2)
    //   + Math.pow(relativePose.getY(), 2)
    // );

    // double speed = distancePid.calculate(distance, 0);
    
    double position = drive.getPosition();
    double speed = distancePid.calculate(position, 1); // 1 meter

    System.out.println("Position: " + Units.metersToInches(position) + "; target: " + Units.inchesToMeters(12) + "; speed: " + speed);

    drive.differentialDrive(speed, 0);


    // if (tick % 20 == 0) {
    //   System.out.println("Relative pose: " + relativePose);
    //   System.out.println("Distance: " + distance);
      
    //   broadcastMessage("Current pose: " + pose + "; Relative pose: " + relativePose);
    //   broadcastMessage("Distance: " + distance + "; adjustment: " + speed);
    // }
  }
}
