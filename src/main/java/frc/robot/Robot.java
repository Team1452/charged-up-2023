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
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
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
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmSubsystem.ArmTargetChoice;
import frc.robot.commands.Balance;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.SetArmAndExtender;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.EditableParameter;
import io.javalin.Javalin;
import kotlin.UByteArrayKt;
import frc.robot.commands.CenterPhotonVisionTarget;

public class Robot extends TimedRobot {
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_1[0], RobotMap.SOLENOID_1[1]);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_2[0], RobotMap.SOLENOID_2[1]);

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // COUNTERCLOCKWISE is positive

  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(1, -1, 0);
  private final SlewRateLimiter armSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);

  private final SlewRateLimiter driveSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);
  private final SlewRateLimiter turnSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);

  private final AnalogInput pressureSensor = new AnalogInput(1);


  private final DriveSubsystem drive = new DriveSubsystem(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_LEFT_INVERTED, RobotMap.MOTOR_RIGHT, RobotMap.MOTOR_RIGHT_INVERTED);

  // Logitech joystick, make sure that Xbox is 0 and Joystick is 1
  private final XboxController controller = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

  private int tick = 0;

  private final ArmSubsystem armSubSys = new ArmSubsystem(RobotMap.MOTOR_ARM, RobotMap.MOTOR_EXTEND);
  private ArmSubsystem.ArmTargetChoice target = ArmTargetChoice.MANUAL_CONTROL;
  private ArmSubsystem.ArmTargetChoice[] targetModes = {
    ArmSubsystem.ArmTargetChoice.MANUAL_CONTROL,
    ArmSubsystem.ArmTargetChoice.LEVEL_THREE_PLATFORM,
    ArmSubsystem.ArmTargetChoice.LEVEL_THREE_POLE,
    ArmSubsystem.ArmTargetChoice.LEVEL_TWO_PLATFORM,
    ArmSubsystem.ArmTargetChoice.LEVEL_TWO_POLE
  };

  private static ShuffleboardTab tab = Shuffleboard.getTab(String.format("Pt. H %.4f", Math.random()));

  private PIDController turnPid = new PIDController(Constants.DriveConstants.kControlTurnP, Constants.DriveConstants.kControlTurnI, Constants.DriveConstants.kControlTurnD);

  private GenericEntry kBalanceP, kBalanceI, kBalanceD;
  private GenericEntry kTurnP, kTurnI, kTurnD;
  private GenericEntry kMaxSpeed, kMaxVoltage;

  private Balance balanceCommand;

  private File balancingPointsLog;
  private FileWriter balancingPointsLogWriter;

  public static EditableParameter kA = new EditableParameter(tab, "Decay Exponential", -0.035);

  @Override
  public void robotInit() {
    turnPid.enableContinuousInput(-180, 180);

    try {
      LocalTime time = LocalTime.now();
      balancingPointsLog = new File(String.format("/home/lvuser/custom/point_%d:%d:%d_%s.csv", (time.getHour() + 1) % 12, time.getMinute(), time.getSecond(), time.getHour() > 11 ? "PM" : "AM"));
      balancingPointsLog.createNewFile();
      balancingPointsLogWriter = new FileWriter(balancingPointsLog);
      balancingPointsLogWriter.write("Pitch,Speed,Turn\n");
      balancingPointsLogWriter.flush();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    balanceCommand = new Balance(drive);
    kBalanceP = tab.add("kBalanceP", Constants.DriveConstants.kBalanceP).getEntry();
    kBalanceI = tab.add("kBalanceI", Constants.DriveConstants.kBalanceI).getEntry();
    kBalanceD = tab.add("kBalanceD", Constants.DriveConstants.kBalanceD).getEntry();
    kTurnP = tab.add("kTurnP", turnPid.getP()).getEntry();
    kTurnI = tab.add("kTurnI", turnPid.getI()).getEntry();
    kTurnD = tab.add("kTurnD", turnPid.getD()).getEntry();
    kMaxSpeed = tab.add("kMaxSpeed", Constants.DriveConstants.kMaxSpeed).getEntry();
    // kMaxVoltage = tab.add("kMaxVoltage", Constants.DriveConstants.kMaxVoltage).getEntry();
    kMaxVoltage = tab.add("kMaxVoltage", 0.35).getEntry();

    compressor.disable();
  }

  @Override
  public void robotPeriodic() {
    drive.updateOdometry();

    Constants.DriveConstants.kBalanceP = kBalanceP.getDouble(0);
    Constants.DriveConstants.kBalanceI = kBalanceI.getDouble(0);
    Constants.DriveConstants.kBalanceD = kBalanceD.getDouble(0);

    turnPid.setP(kTurnP.getDouble(0));
    turnPid.setI(kTurnI.getDouble(0));
    turnPid.setD(kTurnD.getDouble(0));

    drive.setMaxVoltage(kMaxVoltage.getDouble(0));
    drive.setMaxSpeed(kMaxSpeed.getDouble(0));

    Pose2d pose = drive.getPoseWithVisionMeasurements();

    tick += 1;
  }

  Command auton;

  @Override
  public void autonomousInit() {
    armSubSys.reset();
    armSubSys.setIdleMode(CANSparkMax.IdleMode.kCoast);

    Command moveForward = new MoveDistance(10, drive);
    Command moveBackward = new MoveDistance(-3, drive);
    Command balance = new Balance(drive);

    drive.setIdleMode(IdleMode.kBrake);
    // drive.resetPosition(new Pose2d(10, 0, Rotation2d.fromDegrees(-180)));
    drive.resetPositionOdometry();
    drive.disablePIDControl();

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
    // var outOfZone = new SequentialCommandGroup(
    //   // new TurnToAngle(-2.2239612403854787, drive).withTimeout(3.5),
    //   new MoveDistance(Units.metersToInches(1.051658639992556), drive)
    // );
    // outOfZone.schedule();


    var straightBalancingSequence = new SequentialCommandGroup(
        new MoveDistance(10.517001824211251, drive).withTimeout(3.5),
        new MoveDistance(-8.1151787916318057, drive)
          .withPitchExitThreshold(6)
          .withTimeout(3.5),
        new Balance(drive)
    );

    var meterTest = new SequentialCommandGroup(
      new MoveDistance(1, drive)
    );

    var exitCommunity = new SequentialCommandGroup(
      new MoveDistance(3.5, drive)
    );

    Command jerkBack = new MoveDistance(-Units.inchesToMeters(6), drive)
        .withCustomGains(50, 0, 0)
        .withMaxVoltage(0.5)
        .withTimeout(1)
        .andThen(() -> drive.killMotors())
        .andThen(new MoveDistance(Units.inchesToMeters(6), drive))
        .andThen(() -> drive.killMotors());

    SequentialCommandGroup flipAndPushBackCone = new SequentialCommandGroup(
      new SetArmAndExtender(armSubSys, 10, armSubSys.getExtenderPosition()/Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE)
        .withTimeout(2),
      new SetArmAndExtender(armSubSys, 0, armSubSys.getExtenderPosition()/Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE)
        .withTimeout(2)
    ).andThen(() -> drive.killMotors());

    SequentialCommandGroup jerkClimbAndExit = new SequentialCommandGroup(
      jerkBack,
      new MoveDistance(2, drive)
        .withPitchExitThreshold(10)
        .withTimeout(5),
      new Balance(drive).withTimeout(9)
    ).andThen(() -> drive.holdPosition());

    // auton = new Balance(drive).andThen(() -> drive.holdPosition());
    auton = flipAndPushBackCone;
    auton.schedule();


    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxAutonVoltage;
    drive.setMaxVoltage(Constants.DriveConstants.kMaxVoltage);
    // new CenterPhotonVisionTarget(drive).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    // Run currently schedule commands
    CommandScheduler.getInstance().run();

    Pose2d poseOdom = drive.getPoseFromOdometry();
    Pose2d poseOdomAndVision = drive.getPoseWithVisionMeasurements();
  }

  @Override
  public void autonomousExit() {
    armSubSys.setIdleMode(CANSparkMax.IdleMode.kBrake);
    System.out.println("Setting idle mode");
    drive.setIdleMode(IdleMode.kCoast);
    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxDriveVoltage;
    if (auton != null) auton.cancel(); // Cancel if auton didn't terminate properly
  }

  MjpegServer driverServer;
  PhotonCameraWrapper pCam = new PhotonCameraWrapper();
  double teleopTargetAngle = 0;

  @Override
  public void teleopInit() {
    drive.disablePIDControl();
    drive.setIdleMode(IdleMode.kBrake);
    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxDriveVoltage;
    drive.setMaxVoltage(1);
    teleopTargetAngle = drive.getHeading();
    armSubSys.reset();
  }

  @Override
  public void teleopExit() {
    compressor.disable();
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;

  boolean controlMode = true;
  double pressure = 0;
  boolean pistonForward2 = false;


  boolean lastAButtonStatus = false;
  boolean lastBButtonStatus = false;

  boolean loggingPoints = false;
  double scalefac = 1;

  Command scheduledCommand = null;

  boolean turnIsAbsolute = false;

  @Override
  public void teleopPeriodic() {
    Pose2d pose = drive.getPoseWithVisionMeasurements();
    System.out.println("Estimated pose: X: " + pose.getX() + ", Y: " + pose.getY() + ", yaw: " + pose.getRotation().getDegrees());

    double throttle = 1-(joystick.getThrottle() + 1)/2;
    
    double jx = joystick.getX();
    double jy = -joystick.getY();
    double deadzone = 0.02;
    double speed = Math.copySign(Math.max(0, Math.abs(Math.pow(jy, 3.0)) - deadzone), jy);
    double turn;

    if (turnIsAbsolute) {
      teleopTargetAngle = teleopTargetAngle + 10 * jx;
      turn = throttle*turnPid.calculate(teleopTargetAngle);
    } else {
      turn = Math.copySign(Math.max(0, Math.abs(Math.pow(jx, 3.0)) - deadzone), jx);
    }

    drive.differentialDrive(throttle*speed, throttle*turn);

    if (controller.getXButtonPressed()) {
      if (scheduledCommand != null && scheduledCommand.isScheduled()) scheduledCommand.cancel();

      scheduledCommand = new SequentialCommandGroup(
        new TurnToAngle(90, drive),
        new MoveDistance(1, drive),
        new TurnToAngle(0, drive)
      );

      scheduledCommand.schedule();
    }

    if (joystick.getRawButtonPressed(2)) {
      // if (drive.isUsingVelocity()) {
      //   drive.disableVelocityControl();
      //   System.out.println("Robot: Using voltage control");
      // } else {
      //   drive.enableVelocityControl();
      //   System.out.println("Robot: Using velocity control");
      // }
    }

    // Thumb button on top of joystick
    if (joystick.getRawButtonPressed(7)) turnIsAbsolute = !turnIsAbsolute;
    if (joystick.getRawButtonPressed(2)) target = ArmSubsystem.ArmTargetChoice.MANUAL_CONTROL;
    if (joystick.getRawButtonPressed(10)) target = ArmSubsystem.ArmTargetChoice.LEVEL_THREE_PLATFORM;
    if (joystick.getRawButtonPressed(12)) target = ArmSubsystem.ArmTargetChoice.LEVEL_TWO_PLATFORM;
    if (joystick.getRawButtonPressed(9)) target = ArmSubsystem.ArmTargetChoice.LEVEL_THREE_POLE;
    if (joystick.getRawButtonPressed(11)) target = ArmSubsystem.ArmTargetChoice.LEVEL_TWO_POLE;
    //armSubSys.setTargetMode(target);
    if(controller.getRawButton(6)) {
      // if (balanceCommand.isScheduled())
      //   balanceCommand.cancel();
      // else
      //   balanceCommand.schedule();
    }

    //EXTENDER - 3.6 inches for every 5 rotations of the motr
    if(controller.getRightBumper()) {
      armSubSys.changeExtenderPosition(1.2);
    }

    if(controller.getLeftBumper()) {
      armSubSys.changeExtenderPosition(-1.2);
    }

    armSubSys.changeArmPosition(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
    // System.out.println("Extender position: " + armSubSys.getArmEncoder().getPosition());
    // System.out.println("Arm Encoder: " + armSubSys.getArmEncoder().getPosition());
    // System.out.println("Target Arm Position: " + target);
    if (compressorEnabled) {
      // Calculate pressure from analog input
      // (from REV analog sensor datasheet)
      double vOut = pressureSensor.getAverageVoltage();
      double pressure = 250 * (vOut / Constants.PneumaticConstants.ANALOG_VCC) - 15; // Should be 25 but 15 works?

      System.out.print("Compressor enabled; voltage: " + compressor.getAnalogVoltage() + "; estimated pressure: " + pressure);

      if (pressure < Constants.PneumaticConstants.MAX_PRESSURE) {
        System.out.print("; COMPRESSOR IS ON\n");
        compressor.enableDigital();
      } else {
        System.out.print("; COMPRESSOR IS OFF\n");
        compressor.disable();
      }
    } else {
      compressor.disable();
    }

    // if (controller.getRawButtonPressed(7)) {
    if (!lastAButtonStatus && controller.getAButton()) {
      compressorEnabled = !compressorEnabled;
      if (compressorEnabled) {
        System.out.print("COMPRESSOR IS NOW ON\n");
      } else {
        System.out.print("COMPRESSOR IS NOW OFF\n");
      }
    }


    // PISTON
    if (!lastBButtonStatus && controller.getBButtonPressed()) {
      pistonForward = !pistonForward;
      if (pistonForward) {
        System.out.println("CLOSING PISTON");
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kForward);
      } else {
        System.out.println("OPENING PISTON");
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kReverse);
      }
    }

    lastAButtonStatus = controller.getAButton();
    lastBButtonStatus = controller.getBButton();

    CommandScheduler.getInstance().run();
    target = armSubSys.update();
  }


  Pose2d poseAhead;

  @Override
  public void testInit() {
    armSubSys.setIdleMode(IdleMode.kCoast);
  }

  boolean needsToFaceFront = false;

  double lastSetAngle = 0;

  @Override
  public void testPeriodic() {
    double throttle = 1-(joystick.getThrottle() + 1)/2;
    double x = joystick.getX();
    double y = -joystick.getY(); // Forward is negative on joystick

    x = Math.copySign(Math.max(Math.abs(x) - 0.1, 0), x);
    y = Math.copySign(Math.max(Math.abs(y) - 0.1, 0), y);

    double speed = Math.hypot(x, y);
    speed = Math.copySign(Math.pow(speed, 3), speed);

    double targetAngle = -Math.toDegrees(Math.atan2(y, x));
    double currentAngle = drive.getHeading();

    if (Math.abs(targetAngle - lastSetAngle) > 10) {
      turnPid.reset();
    }

    if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
      speed = 0;
      targetAngle = lastSetAngle;
    }

    lastSetAngle = targetAngle;

    double turn = turnPid.calculate(currentAngle, targetAngle);

    if (joystick.getRawButtonPressed(2)) {
      needsToFaceFront = !needsToFaceFront;
    }

    if (joystick.getTrigger()) {
      speed = speed * 0.1;
    } else {
      speed = 0;
    }

    System.out.printf("Current angle: %.3f. Target angle: %.3f\n", currentAngle, targetAngle);

    drive.differentialDrive(speed, throttle * turn);
  }
}
