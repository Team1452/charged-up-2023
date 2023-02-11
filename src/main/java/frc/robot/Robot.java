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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.Vec2;
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
  private Javalin app = null;

  RamseteController ramsete = new RamseteController(1, 0.5);
  PIDController turnPid = new PIDController(0.03, 0.01, 0); // 0.002, 0.01);

  PIDController distancePid = new PIDController(0.2, 0, 0.002);

  boolean targetingIntermediaryTranslation = true;

  ShuffleboardTab tab = Shuffleboard.getTab("AprilTag Test");

  private final Map<WsContext, String> clients = new ConcurrentHashMap<>();
  private int userNumber = 0;

  public void initWS() {
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

  @Override
  public void robotInit() {
    tick = 0;
    drive = new Drivetrain(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);

    // initWS();
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
    var speed = Math.pow(-controller.getLeftY(), 3);
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

    drive.resetPosition();
  }

  String poseToString(Pose2d pose) {
    return "X: " + pose.getX() + ", Y: " + pose.getY() 
      + ", yaw: " + pose.getRotation().getDegrees();
  }



  @Override
  public void testPeriodic() {
    drive.updateOdometry();
    Pose2d pose = drive.getPose();
    Pose2d relativePose = pose.relativeTo(PhotonCameraWrapper.tag04.pose.toPose2d());
    // Pose2d targetPose = poseAhead;

    // // Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(90)));

    // Pose2d relativePose = pose.relativeTo(targetPose);

    // double distance = Math.sqrt(
    //   Math.pow(relativePose.getX(), 2)
    //   + Math.pow(relativePose.getY(), 2)
    // );

    // double speed = distancePid.calculate(distance, 0);
    
    double relativeX = relativePose.getX();
    double relativeY = relativePose.getY();
    double relativeLength = Math.sqrt(relativeX*relativeX + relativeY*relativeY);

    // double dotProd = new Vec2(relativePose.getX(), relativePose.getY())
    //   .hat()
    //   .dot(new Vec2(pose.getRotation().getCos(), pose.getRotation().getSin()));

    Rotation2d currentRotation = pose.getRotation();

    Vec2 currentHeading = new Vec2(currentRotation.getCos(), currentRotation.getSin());
    Vec2 targetHeading = new Vec2(relativePose.getX(), relativePose.getY()).hat();

    // Dot product tells us how far we are from target angle
    double dotProd = currentHeading.dot(targetHeading);
    
    // Use z component of 3d cross product to know whether
    // to turn clockwise or counterclockwise
    double crossZ = currentHeading.getX() * targetHeading.getY()
      - currentHeading.getY() * targetHeading.getX();

    double turn = Math.signum(crossZ) * turnPid.calculate(dotProd, 1);

    turnPid.setTolerance(0.02);

    System.out.print("Can see AprilTag: ");

    for (var target : drive.getPcw().getTargets()) {
      System.out.print(target.getFiducialId() + " ");
    }

    System.out.print("; Current dot product: " + dotProd + ", turning: "  + turn + ", cross: " + crossZ + "\n");

    turnPid.setIntegratorRange(-0.1, 0.1);

    if (!turnPid.atSetpoint()) {
      drive.differentialDrive(0, turn);
    } else {
      // Negate because behind apriltag
      double speed = -distancePid.calculate(relativeLength, 1); // 1 meter from AprilTag

      System.out.println("Distance: " + Units.metersToInches(relativeLength) + "; target: " +
        Units.metersToInches(1) + "; speed: " + speed);

      drive.differentialDrive(speed, 0);
    }
    
    // double position = drive.getPosition();



    // if (tick % 20 == 0) {
    //   System.out.println("Relative pose: " + relativePose);
    //   System.out.println("Distance: " + distance);
      
    //   broadcastMessage("Current pose: " + pose + "; Relative pose: " + relativePose);
    //   broadcastMessage("Distance: " + distance + "; adjustment: " + speed);
    // }
  }
}
