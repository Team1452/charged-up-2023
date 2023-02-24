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
import java.util.Random;
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
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.commands.MoveToPose;
import frc.robot.commands.MoveToPoseCurved;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.Vec2;
import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import org.json.*;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
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

  ShuffleboardTab tab = Shuffleboard.getTab("AprilTag Turning Test " + Math.random());

  private final Map<WsContext, String> clients = new ConcurrentHashMap<>();
  private int userNumber = 0;

  Command currentMoveToPoseCommand = null;

  private Joystick joystick = new Joystick(0);

  public void initWS() {
    app = Javalin.create().start(7070);
  
    app.ws("/debug", ws -> {
      ws.onConnect(ctx -> {
        String username = "User" + userNumber++;
        clients.put(ctx, username);
      });

      ws.onMessage(msg -> {
        String message = msg.message();
        System.out.println("Got message: " + message);
        JSONObject json = new JSONObject(message);

        double targetYaw = json.getDouble("yaw");

        JSONArray position = json.getJSONArray("position");
        double targetX = Units.inchesToMeters(position.getDouble(0));
        double targetY = Units.inchesToMeters(position.getDouble(1));

        Pose2d targetPose = new Pose2d(targetX, targetY, new Rotation2d(Math.toRadians(targetYaw)));
        Pose2d currentPose = drive.getPose();
        System.out.println("Got target pose: " + targetPose + "; current pose " + currentPose);

        // System.out.println("turning to angle " + facingAngle + " deg, moving " + distance + " inches");
        // Command sequence = new SequentialCommandGroup(
        //   // new TurnToAngle(facingAngle, drive),
        //   // new MoveDistance(distance, drive),
        //   new TurnToAngle(targetPose.getRotation().getDegrees(), drive)
        // ).withTimeout(5); // Kill if can't complete in 5s

        // Command moveToPoseCurved = new MoveToPoseCurved(targetPose, drive, this);
        Command moveToPose = new MoveToPose(targetPose, drive);

        moveToPose.schedule();
        // if (currentMoveToPoseCommand != null)
        //   currentMoveToPoseCommand.cancel();
        // currentMoveToPoseCommand = new MoveToPose(targetPose, drive);
        // currentMoveToPoseCommand.schedule();
      });

      ws.onClose(ctx -> {
        clients.remove(ctx);
      });
    });
  }

  public GenericEntry turningPidP, turningPidI, turningPidD;
  public GenericEntry distancePidP, distancePidI, distancePidD;

  @Override
  public void robotInit() {
    tick = 0;
    drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);

    targetAngleInput = tab.add("Target Angle (Deg)", 0).getEntry();

    turningPidP = tab.add("Turning PID P", 0.01).getEntry();
    turningPidI = tab.add("Turning PID I", 0.001).getEntry();
    turningPidD = tab.add("Turning PID D", 0.0001).getEntry();

    distancePidP = tab.add("Distance PID P", 0.1).getEntry();
    distancePidI = tab.add("Distance PID I", 0.01).getEntry();
    distancePidD = tab.add("Distance PID D", 0.001).getEntry();

    // initWS();
  }

  private void broadcastMessage(String message) {
    for (WsContext ctx : clients.keySet()) {
      ctx.send(message);
    }
  }

  @Override
  public void robotPeriodic() {
    drive.updateOdometry();
    tick += 1;
    if (tick % 20 == 0) {
      Pose2d pose = drive.getPose();
      System.out.println("Angle: " + pose.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(pose.getX()) + ", Y: " + Units.metersToInches(pose.getY()));
    }

  }

  @Override
  public void autonomousInit() {
    Command moveForward = new MoveDistance(10, drive);
    Command moveBackward = new MoveDistance(-3, drive);
    Command balance = new Balance(drive);

    drive.setIdleMode(IdleMode.kBrake);
    drive.resetPosition(new Pose2d(0, 0, new Rotation2d(Math.toRadians(-180))));

    Command followRectangle = new SequentialCommandGroup(
      // new TurnToAngle(-180, drive),
      // new MoveDistance(Units.inchesToMeters(80), drive),

      // new TurnToAngle(-90, drive),
      // new MoveDistance(Units.inchesToMeters(60), drive),

      // new TurnToAngle(0, drive),
      // new MoveDistance(Units.inchesToMeters(80), drive),

      // new TurnToAngle(90, drive),
      // new MoveDistance(Units.inchesToMeters(60), drive),

      // new TurnToAngle(180, drive)

      new TurnToAngle(0, drive)

    );

    // Command turningTest = new TurnToAngle(-90, drive)
    //   .andThen(new WaitCommand(1))
    //   .andThen(new TurnToAngle(-180, drive))
    //   .andThen(new WaitCommand(1))
    //   .andThen(new TurnToAngle(-270, drive))
    //   .andThen(new WaitCommand(1))
    //   .andThen(new TurnToAngle(0, drive))
    //   .andThen(drive::resetPosition)
    //   .andThen(followRectangle);


    // Command goToPointAndFollowRectangle = new MoveToPose(new Pose2d(-10, -10, new Rotation2d(0)), drive);

    // System.out.println("Scheduling command");
    followRectangle.schedule();



    // Command moveForward = new MoveDistance(10, drive);
    // Command moveBackward = new MoveDistance(-3, drive);
    // Command balance = new Balance(drive);

    // moveForward
    //   .andThen(moveBackward)
    //   .andThen(balance);
  }

  @Override
  public void autonomousPeriodic() {
    // Run currently scheduled commands
    drive.updateOdometry();
    
    // if (joystick.getRawButton(2)) {
      CommandScheduler.getInstance().run();
    // }

    Pose2d pose = drive.getPose();

    if (tick % 5 == 0) {
      // String json = String.format("{"
      //   + "\"position\": [%f, %f],"
      //   + "\"yaw\": %f"
      //   + "}",
      //   Units.metersToInches(pose.getX()),
      //   Units.metersToInches(pose.getY()),
      //   pose.getRotation().getRadians()
      // );
      // broadcastMessage(json);
      // System.out.println("Angle: " + pose.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(pose.getX()) + ", Y: " + Units.metersToInches(pose.getY()));
    }

  }

  @Override
  public void autonomousExit() {
    System.out.println("Setting idle mode");
    if (currentMoveToPoseCommand != null) {
      currentMoveToPoseCommand.cancel();
    }
    drive.differentialDrive(0, 0);
    drive.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void teleopPeriodic() {
    double speed = -joystick.getY();
    double turn = joystick.getX();

    speed = Math.copySign(Math.pow(speed, 3), speed);
    turn = Math.copySign(Math.pow(turn, 3), turn);

    drive.differentialDrive(speed, turn);

    // Maybe switch to SlewRateLimiter or PID for smoother control?
    // var speed = Math.pow(-controller.getLeftY(), 3);
    // var rot = Math.pow(controller.getRightX(), 3);

    // // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

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
    Pose2d pose = drive.getPose();
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
