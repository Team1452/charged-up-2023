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
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.StreamReadCapability;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.Balance;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.CenterPhotonVisionTarget;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_1[0], RobotMap.SOLENOID_1[1]);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_2[0], RobotMap.SOLENOID_2[1]);

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private final BangBangController pressureController = new BangBangController();

  // COUNTERCLOCKWISE is positive
  private final CANSparkMax arm = new CANSparkMax(RobotMap.MOTOR_ARM, MotorType.kBrushless);
  // private final CANSparkMax extender = new CANSparkMax(RobotMap.MOTOR_EXTEND, MotorType.kBrushless);

  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(1, -1, 0);
  private final SlewRateLimiter armSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);

  private final SlewRateLimiter driveSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);
  private final SlewRateLimiter turnSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);

  private final AnalogInput pressureSensor = new AnalogInput(0);

  private SparkMaxPIDController armPID;
  private SparkMaxPIDController extenderPID;

  private DriveSubsystem drive;

  private final RelativeEncoder armEncoder = arm.getEncoder();
  private final RelativeEncoder extenderEncoder = arm.getEncoder();

  private final Joystick joystick = new Joystick(0);

  private Command balancingCommand = null;
  private Command centeringCommand = null;
  private Command turningCommand = null;

  private int tick = 0;

  double armAngle;
  double extenderPosition;

  public Robot() {
    super(Constants.PERIOD_MS/1000);
  }

  @Override
  public void robotInit() {
    compressor.disable();
    
    if (RobotMap.USING_TESTBED) {
      // Testbed
      drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_LEFT_INVERTED, RobotMap.TEST_MOTOR_RIGHT, RobotMap.TEST_MOTOR_RIGHT_INVERTED);
    } else {
      // Competition robot
      drive = new DriveSubsystem(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_LEFT_INVERTED, RobotMap.MOTOR_RIGHT, RobotMap.MOTOR_RIGHT_INVERTED);
    }

    balancingCommand = new Balance(drive);
    centeringCommand = new CenterPhotonVisionTarget(drive);

    // arm.restoreFactoryDefaults();
    // arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // // extender.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // // armEncoder = arm.getEncoder();
    // // extenderEncoder = arm.getEncoder();
    // extenderEncoder.setPosition(0);
    // armEncoder.setPosition(0);
    // armPID = arm.getPIDController();
    // armAngle = armEncoder.getPosition();

    // extenderEncoder.setPosition(0);
    // // extenderPID = extender.getPIDController();
    // extenderPosition = extenderEncoder.getPosition();

    // armPID.setP(0.1);
    // armPID.setI(0.0001);
    // armPID.setD(0.001);
    // armPID.setOutputRange(-1, 1);
    // armPID.setIZone(0);
    // armPID.setFF(0);

    // // sets absolute encoder limits for arm
    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ArmConstants.MIN_ROTATION);
    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.ArmConstants.MAX_ROTATION);
    // // armPID.setReference(0, CANSparkMax.ControlType.kPosition);

    // extenderPID.setP(0.1);
    // extenderPID.setI(0);
    // extenderPID.setD(0);
    // extenderPID.setOutputRange(-1, 1);
    // extenderPID.setIZone(0);
    // extenderPID.setFF(0);

    // extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ExtenderConstants.MAX_EXTENDER_ROTATION);
    // extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  }

  @Override
  public void robotPeriodic() {
    tick += 1;
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
      new TurnToAngle(90, drive),
      new WaitCommand(1),
      new TurnToAngle(180, drive),
      new WaitCommand(1),
      new TurnToAngle(270, drive),
      new WaitCommand(1),
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
      new MoveDistance(Units.inchesToMeters(20), drive),
      
      new TurnToAngle(90, drive),
      new MoveDistance(Units.inchesToMeters(10), drive),

      new TurnToAngle(180, drive),
      new MoveDistance(Units.inchesToMeters(20), drive),

      new TurnToAngle(270, drive),
      new MoveDistance(Units.inchesToMeters(10), drive),

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
    // rotateAngles.schedule();

    // new SequentialCommandGroup(
    //   new MoveDistance(1, drive),
    //   new TurnToAngle(-90, drive),
    //   new MoveDistance(1, drive),
    //   new TurnToAngle(-180, drive),
    //   new MoveDistance(1, drive)
    // ).schedule();

    new CenterPhotonVisionTarget(drive).schedule();

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
    // if (tick % 2 == 0) {
    //   System.out.println("ODOMETRY ONLY: Angle: " + poseOdom.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(poseOdom.getX()) + ". Y: " + Units.metersToInches(poseOdom.getY()));
    //   System.out.println("ODOMETRY + VISION: Angle: " + poseOdomAndVision.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(poseOdomAndVision.getX()) + ". Y: " + Units.metersToInches(poseOdomAndVision.getY()));
    // }
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
    
    // No flip for actual robot
    drive.differentialDrive(speed, rot); // Flip CW/CCW

    if (controller.getRightBumperPressed()) {
      centeringCommand.schedule();
    }

    if (controller.getYButtonPressed()) {
      if (turningCommand != null) turningCommand.cancel();
      turningCommand = new TurnToAngle(0, drive);
      turningCommand.schedule();
    }

    if (controller.getBButtonPressed()) {
      if (turningCommand != null) turningCommand.cancel();
      turningCommand = new TurnToAngle(90, drive);
      turningCommand.schedule();
    }

    if (controller.getXButtonPressed()) {
      if (turningCommand != null) turningCommand.cancel();
      turningCommand = new TurnToAngle(-90, drive);
      turningCommand.schedule();
    }

    if (controller.getLeftBumperPressed()) {
      CommandScheduler.getInstance().cancelAll();
    }

    if (controller.getAButtonPressed()) {
      if (balancingCommand.isScheduled()) {
        System.out.println("Killing balance");
        balancingCommand.cancel();
      } else {
        System.out.println("Balancing");
        balancingCommand.schedule();
      }
    }

    if (joystick.getTrigger()) {
      CommandScheduler.getInstance().run();
    }

    var gyro = drive.getGyro();
    // System.out.println("Yaw: " + gyro.getYaw() + ", pitch: " + gyro.getPitch() + ", roll: " + gyro.getRoll());

    // arm.set(controller.getLeftY());

    // if (controller.getAButtonPressed()) {
    //   solenoid.set(Value.kForward);
    // } else if (controller.getAButtonReleased()) {
    //   solenoid.set(Value.kReverse);
    // }
    CommandScheduler.getInstance().run();
  }


  Pose2d poseAhead;

  @Override
  public void testInit() {
    leftSolenoid.set(Value.kOff); // Off by default
    rightSolenoid.set(Value.kOff);
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;

  boolean controlMode = true;
  double pressure = 0;
  boolean pistonForward2 = false;

  @Override
  public void testPeriodic() {
    
    // double joystickThrottle = joystick.getThrottle();
    // boolean joystickTrigger = joystick.getTrigger();
    // double pov = joystick.getPOV();

    // System.out.println("throttle: " + joystickThrottle + "; trigger: " + joystickTrigger + "; pov: " + pov);

    // DRIVE

    // Controller forward is negative
    double speed = Math.pow(-controller.getLeftY(), 3.0);
    double turn = Math.pow(controller.getLeftX(), 3.0);

    // System.out.println("Joystick y: " + joystick.getY() + " ; joystick x: " + joystick.getX());
    // double joystickY = Math.signum(joystick.getY()) * Math.max(0, Math.abs(joystick.getY()) - 0.1);
    // double joystickX = Math.signum(joystick.getX()) * Math.max(0, Math.abs(joystick.getX()) - 0.1);
    // speed = driveSpeedLimiter.calculate(-joystickY);
    // turn = turnSpeedLimiter.calculate(joystickX);
    // System.out.println("Speed: " + speed + "; turn: " + turn);

    drive.differentialDrive(speed, -turn);

    // EXTENDER

    // TODO: Change to PID
    extenderPosition += (controller.getLeftTriggerAxis()
        - controller.getRightTriggerAxis()) * 0.1;
    extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);
    System.out.println("Extender position: " + extenderPosition);

    // ARM
    if (controller.getLeftBumper()) {
      // Turn counterclockwise
      armAngle += 0.3;
    }

    if (controller.getRightBumper()) {
      // Turn clockwise
      armAngle -= 0.3;
    }

    if (controller.getXButtonPressed()) {
      // Align arm with level two game node height
      double armLength = Constants.ExtenderConstants.MIN_ARM_LENGTH
          + armEncoder.getPosition() * Constants.ExtenderConstants.METERS_PER_ROTATION;
      double angle = Math
          .atan((Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT - Constants.ArmConstants.ARM_HEIGHT) / armLength);
      System.out.println("armLength: " + armLength);
      System.out.println("arm angle: " + angle);
      armAngle = Constants.ArmConstants.ARM_GEARING * angle;
    }

    armPID.setReference(armAngle, CANSparkMax.ControlType.kPosition);

    // COMPRESSOR
    if (compressorEnabled) {
      // Calculate pressure from analog input
      // (from REV analog sensor datasheet)
      double vOut = pressureSensor.getAverageVoltage();
      double pressure = 250 * (vOut / Constants.PneumaticConstants.ANALOG_VCC) - 25;

      pressureController.calculate(pressure, 60);
      if (pressureController.atSetpoint()) {
        compressor.enableDigital();
      } else {
        compressor.disable();
      }
    } else {
      compressor.disable();
    }

    if (controller.getAButtonPressed()) {
      compressorEnabled = !compressorEnabled;
    }

    // PISTON
    if (controller.getYButtonPressed()) {
      pistonForward = !pistonForward;
      System.out.println("Enabling solenoid: " + pistonForward);
      if (pistonForward) {
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kForward);
      } else {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kReverse);
      }
    }
  }
}
