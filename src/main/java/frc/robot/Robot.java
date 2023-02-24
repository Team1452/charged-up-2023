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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Balance;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.Vec2;
import io.javalin.Javalin;
import io.javalin.websocket.WsContext;

public class Robot extends TimedRobot {
  // private final XboxController controller = new XboxController(0);
  private Joystick joystick = new Joystick(0);
  private DriveSubsystem drive;
  private int tick;
  private final PIDController balancer = new PIDController(0.001, 0, 0);
  // private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  // private final DoubleSolenoid solenoid = new DoubleSolenoid(
    // PneumaticsModuleType.REVPH, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);
  private Javalin app = null;

  RamseteController ramsete = new RamseteController(1, 0.5);
  PIDController turnPid = new PIDController(0.1, 0.02, 0); // 0.002, 0.01);

  PIDController distancePid = new PIDController(0.1, 0, 0.002);

  boolean targetingIntermediaryTranslation = true;

  ShuffleboardTab tab = Shuffleboard.getTab("AprilTag Turning Test");

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
    drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);

    targetAngleInput = tab.add("Target Angle (Deg)", 0).getEntry();

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

    drive.updateOdometry();

  }

  @Override
  public void autonomousInit() {
    Command moveForward = new MoveDistance(10, drive);
    Command moveBackward = new MoveDistance(-3, drive);
    Command balance = new Balance(drive);

    drive.setIdleMode(IdleMode.kBrake);
    // drive.resetPosition(new Pose2d(10, 0, Rotation2d.fromDegrees(-180)));
    drive.resetPositionOdometry();

    Command rotateAngles = new SequentialCommandGroup(
      new TurnToAngle(0, drive),
      new WaitCommand(5),
      new TurnToAngle(90, drive),
      new WaitCommand(5),
      new TurnToAngle(180, drive),
      new WaitCommand(5),
      new TurnToAngle(270, drive),
      new WaitCommand(5),
      new TurnToAngle(360, drive)
    );

    Command followRectangle = new SequentialCommandGroup(
      new TurnToAngle(-180, drive),
      new MoveDistance(Units.inchesToMeters(80), drive),
      
      new TurnToAngle(-90, drive),
      new MoveDistance(Units.inchesToMeters(60), drive),

      new TurnToAngle(0, drive),
      new MoveDistance(Units.inchesToMeters(80), drive),

      new TurnToAngle(90, drive),
      new MoveDistance(Units.inchesToMeters(60), drive),

      // Doesn't converge? TODO: Figure out why
      new TurnToAngle(180, drive)
    );

    Command followRectangleOdom = new SequentialCommandGroup(
      new TurnToAngle(0, drive),
      new MoveDistance(Units.inchesToMeters(80), drive),
      
      new TurnToAngle(90, drive),
      new MoveDistance(Units.inchesToMeters(60), drive),

      new TurnToAngle(180, drive),
      new MoveDistance(Units.inchesToMeters(80), drive),

      new TurnToAngle(270, drive),
      new MoveDistance(Units.inchesToMeters(60), drive),

      new TurnToAngle(0, drive)
    );
    
    
    Command _followRectangleOld = new MoveDistance(Units.inchesToMeters(80), drive)
      .andThen(new TurnToAngle(-90, drive))
      .andThen(new MoveDistance(Units.inchesToMeters(40), drive))
      .andThen(new TurnToAngle(-180, drive))
      .andThen(new MoveDistance(Units.inchesToMeters(80), drive))
      .andThen(new TurnToAngle(-270, drive))
      .andThen(new MoveDistance(Units.inchesToMeters(40), drive))
      .andThen(new TurnToAngle(0, drive));

    System.out.println("Scheduling command");
    // sequence.schedule();

    // rotateAngles.schedule();
    followRectangle.schedule();
    // followRectangleOdom.schedule();



    // Command moveForward = new MoveDistance(10, drive);
    // Command moveBackward = new MoveDistance(-3, drive);
    // Command balance = new Balance(drive);

    // moveForward
    //   .andThen(moveBackward)
    //   .andThen(balance);
  }

  @Override
  public void autonomousPeriodic() {
    // Run currently schedule commands
    CommandScheduler.getInstance().run();

    Pose2d poseOdom = drive.getPoseFromOdometry();
    Pose2d poseOdomAndVision = drive.getPoseWithVisionMeasurements();

    // Every 40ms
    if (tick % 2 == 0) {
      System.out.println("ODOMETRY ONLY: Angle: " + poseOdom.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(poseOdom.getX()) + ". Y: " + Units.metersToInches(poseOdom.getY()));
      System.out.println("ODOMETRY + VISION: Angle: " + poseOdomAndVision.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(poseOdomAndVision.getX()) + ". Y: " + Units.metersToInches(poseOdomAndVision.getY()));
    }

  }

  @Override
  public void autonomousExit() {
    System.out.println("Setting idle mode");
    drive.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void teleopPeriodic() {
    // Maybe switch to SlewRateLimiter or PID for smoother control?
    var speed = Math.pow(-joystick.getY(), 3);
    var rot = Math.pow(joystick.getX(), 3);

    // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);
    drive.differentialDrive(speed, -rot); // Flip CW/CCW

    if (joystick.getTriggerPressed()) {
      System.out.println("Trigger pressed, turning to 180 deg");
      new TurnToAngle(180, drive).schedule();
    }

    if (joystick.getTrigger()) {
      CommandScheduler.getInstance().run();
    }

    // arm.set(controller.getLeftY());

    // if (controller.getAButtonPressed()) {
    //   solenoid.set(Value.kForward);
    // } else if (controller.getAButtonReleased()) {
    //   solenoid.set(Value.kReverse);
    // }
  }


  Pose2d poseAhead;
  GenericEntry targetAngleInput;

  @Override
  public void testInit() {
    Pose2d initialPose = drive.getPoseFromOdometry();
    double yaw = initialPose.getRotation().getRadians();
    poseAhead = new Pose2d(
      initialPose.getX() + Math.cos(yaw) * Units.inchesToMeters(10),
      initialPose.getY() + Math.sin(yaw) * Units.inchesToMeters(10),
      initialPose.getRotation()
    );

    // drive.resetPosition();

  }

  String poseToString(Pose2d pose) {
    return "X: " + pose.getX() + ", Y: " + pose.getY() 
      + ", yaw: " + pose.getRotation().getDegrees();
  }

  @Override
  public void testPeriodic() {
    drive.updateOdometry();

    Pose2d pose = drive.getPoseFromOdometry();
    Pose2d aprilTagPose = PhotonCameraWrapper.tag04.pose.toPose2d();

    Rotation2d currentRotation = pose.getRotation();

    double currentAngle = currentRotation.getRadians();
    double targetAngle = (Math.atan2(aprilTagPose.getY() - pose.getY(), aprilTagPose.getX() - pose.getX()));

    // targetAngle = Math.toRadians(targetAngleInput.getDouble(0));

    // Radians increase CCW, but positive rotation is CW
    turnPid.enableContinuousInput(-Math.PI, Math.PI);
    double turn = turnPid.calculate(currentAngle, targetAngle);

    System.out.println("Current angle: " + Math.toDegrees(currentAngle) + " deg; target angle: " + Math.toDegrees(targetAngle) + " deg; turn: " + turn);

    turn = Math.signum(turn) * Math.min(Math.abs(turn), 0.1); // Stop gap for death spirals

    drive.differentialDrive(0, turn);

    // turnPid.setTolerance(0.1);


    // if (!turnPid.atSetpoint()) {
    // } else {
    //   double distance = Math.hypot(aprilTagPose.getX() - pose.getX(), aprilTagPose.getY() - pose.getY());

    //   // Negate because behind apriltag
    //   double speed = distancePid.calculate(distance, 1); // 1 meter from AprilTag

    //   System.out.println("Distance: " + Units.metersToInches(distance) + "; target: " +
    //     Units.metersToInches(1) + "; speed: " + speed);

    //   drive.differentialDrive(speed, 0);
    // }

    // double position = drive.getPosition();



    // if (tick % 20 == 0) {
    //   System.out.println("Relative pose: " + relativePose);
    //   System.out.println("Distance: " + distance);
      
    //   broadcastMessage("Current pose: " + pose + "; Relative pose: " + relativePose);
    //   broadcastMessage("Distance: " + distance + "; adjustment: " + speed);
    // }
  }
}
