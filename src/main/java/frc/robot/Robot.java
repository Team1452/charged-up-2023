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
import java.util.Random;
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
import edu.wpi.first.hal.communication.NIRioStatus;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ArmSubsystem.ArmTargetChoice;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.DriveSubsystem.ControlMode;
import frc.robot.commands.Balance;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArmAndExtender;
import frc.robot.commands.SetExtender;
import frc.robot.commands.DynamicCommand;
import frc.robot.commands.Lambda;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.EditableParameter;
import frc.robot.util.Utils;
import frc.robot.util.XboxButtonHelper;
import io.javalin.Javalin;
import kotlin.UByteArrayKt;
import frc.robot.commands.CenterPhotonVisionTarget;

public class Robot extends TimedRobot {
  private DigitalInput armLimitSwitch = new DigitalInput(0);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_1[0], RobotMap.SOLENOID_1[1]);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_2[0], RobotMap.SOLENOID_2[1]);

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private final CANSparkMax intake = new CANSparkMax(RobotMap.INTAKE, MotorType.kBrushed);

  // COUNTERCLOCKWISE is positive

  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(1, -1, 0);
  private final SlewRateLimiter armSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);

  private final SlewRateLimiter driveSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);
  private final SlewRateLimiter turnSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);

  private final AnalogInput pressureSensor = new AnalogInput(1);

  private final DriveSubsystem drive = new DriveSubsystem(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_LEFT_INVERTED,
      RobotMap.MOTOR_RIGHT, RobotMap.MOTOR_RIGHT_INVERTED);

  // Logitech joystick, make sure that Xbox is 0 and Joystick is 1
  private final XboxController controller = new XboxController(0);
  private final XboxController driveController = new XboxController(1);

  private final XboxButtonHelper driveControllerButtons = new XboxButtonHelper(driveController);

  private int tick = 0;

  private final ArmSubsystem armSubSys = new ArmSubsystem(RobotMap.MOTOR_ARM, RobotMap.MOTOR_EXTEND, armLimitSwitch);
  // private ArmSubsystem.ArmTargetChoice target = ArmTargetChoice.MANUAL_CONTROL;
  // private ArmSubsystem.ArmTargetChoice[] targetModes = {
  //     ArmSubsystem.ArmTargetChoice.MANUAL_CONTROL,
  //     ArmSubsystem.ArmTargetChoice.LEVEL_THREE_PLATFORM,
  //     ArmSubsystem.ArmTargetChoice.LEVEL_THREE_POLE,
  //     ArmSubsystem.ArmTargetChoice.LEVEL_TWO_PLATFORM,
  //     ArmSubsystem.ArmTargetChoice.LEVEL_TWO_POLE
  // };

  private static ShuffleboardTab tab = Shuffleboard.getTab(String.format("Pt. H %.4f", Math.random()));

  private PIDController turnPid = new PIDController(Constants.DriveConstants.kControlTurnP,
      Constants.DriveConstants.kControlTurnI, Constants.DriveConstants.kControlTurnD);

  private GenericEntry kBalanceP, kBalanceI, kBalanceD;
  private GenericEntry kTurnP, kTurnI, kTurnD;
  private GenericEntry kMaxSpeed, kMaxVoltage;

  private Balance balanceCommand;

  private File balancingPointsLog;
  private FileWriter balancingPointsLogWriter;

  public static EditableParameter armCurrentLimit = new EditableParameter(tab, "Arm Current Limit", CurrentLimits.ARM_LIMIT);
  public static EditableParameter stepSizeDegrees = new EditableParameter(tab, "Step Size Degrees", CalibrateArm.STEP_SIZE_DEGREES);

  public static EditableParameter kA = new EditableParameter(tab, "Decay Exponential", -0.04);

  public static EditableParameter turnIsLinearThreshold = new EditableParameter(tab, "Turn Is Linear Threshold", 0.0, false);

  public static EditableParameter turnCurveScalingFactor = new EditableParameter(tab, "Turn Scaling", 0.4, false);
  public static EditableParameter velocityCurveScalingFactor = new EditableParameter(tab, "Velocity Scaling", 1, false);
  public static EditableParameter turnCurveExponent = new EditableParameter(tab, "Turn Exponent", 2, false);
  public static EditableParameter velocityCurveExponent = new EditableParameter(tab, "Velocity Exponent", 2, false);

  public static EditableParameter speedDeadzone = new EditableParameter(tab, "Speed Deadzone", 0.05, false);
  public static EditableParameter armScale = new EditableParameter(tab, "Arm Scale", 1, false);
  public static EditableParameter armFineScale = new EditableParameter(tab, "Arm Fine Scale", 0.5, false);
  public static EditableParameter r = new EditableParameter(tab, "r", 0.45, false);
  public static EditableParameter z = new EditableParameter(tab, "z", 16, false);
  public static EditableParameter m = new EditableParameter(tab, "m", .3, false);

  public static EditableParameter currentLimitClaw = new EditableParameter(tab, "Current Limit Claw", 80, false);

  public static EditableParameter turnDeadzone = new EditableParameter(tab, "Turn Deadzone", 0.01, false);
  public static EditableParameter turnScale = new EditableParameter(tab, "Default Turn Scale", 0.2, false);
  public static EditableParameter fineScale = new EditableParameter(tab, "Fine Control Turn Scale", 0.1, false);

  public static EditableParameter ArmP = new EditableParameter(tab, "ArmP", 0.12, false);
  public static EditableParameter ArmI= new EditableParameter(tab, "ArmI", 0, false);
  public static EditableParameter ArmD = new EditableParameter(tab, "ArmD",0, false);

  public static EditableParameter extenderP = new EditableParameter(tab, "ExtenderP", 0.3, false);
  public static EditableParameter extenderI = new EditableParameter(tab, "ExtenderI", 0, false);
  public static EditableParameter extenderD = new EditableParameter(tab, "ExtenderD",0, false);

  public static EditableParameter intakeSpeed = new EditableParameter(tab, "Intake Speed", 0.4, false);

  public static EditableParameter driveAveragingCoeff = new EditableParameter(tab, "Drive Averaging Coeff", DriveSubsystem.driveAveragingCoeff);
  public static EditableParameter turnAveragingCoeff = new EditableParameter(tab, "Turn Averaging Coeff", DriveSubsystem.turnAveragingCoeff);
  public static EditableParameter driveTurnCoeffThreshold = new EditableParameter(tab, "Drive Turn Threshold", DriveSubsystem.driveTurnCoeffThreshold);

  // public static DashboardServer server = new DashboardServer();

  public GenericEntry turningPidP, turningPidI, turningPidD;
  public GenericEntry distancePidP, distancePidI, distancePidD;

  @Override
  public void robotInit() {
    // intake.setSmartCurrentLimit(Constants.CurrentLimits.INTAKE_LIMIT);
    turnPid.enableContinuousInput(-180, 180);

    // Add front camera
    // tab.addCamera("Arm Camera", Constants.VisionConstants.armCameraName, "photonvision.local:1182");

    try {
      LocalTime time = LocalTime.now();
      balancingPointsLog = new File(String.format("/home/lvuser/custom/point_%d:%d:%d_%s.csv",
          (time.getHour() + 1) % 12, time.getMinute(), time.getSecond(), time.getHour() > 11 ? "PM" : "AM"));
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
    // kMaxVoltage = tab.add("kMaxVoltage",
    // Constants.DriveConstants.kMaxVoltage).getEntry();
    kMaxVoltage = tab.add("kMaxVoltage", 1.0).getEntry();

    compressor.disable();
  }

  @Override
  public void robotPeriodic() {
    drive.updateOdometry();
    drive.updateVelocityControl();

    DriveSubsystem.driveAveragingCoeff = driveAveragingCoeff.getValue();
    DriveSubsystem.turnAveragingCoeff = turnAveragingCoeff.getValue();
    DriveSubsystem.driveTurnCoeffThreshold = driveTurnCoeffThreshold.getValue();

    Constants.CurrentLimits.ARM_LIMIT = armCurrentLimit.getValue();
    CalibrateArm.STEP_SIZE_DEGREES = stepSizeDegrees.getValue();

    CommandScheduler.getInstance().run();

    armSubSys.getArmPID().setP(extenderP.getValue());
    armSubSys.getArmPID().setI(extenderI.getValue());
    armSubSys.getArmPID().setD(extenderD.getValue());

    Constants.DriveConstants.kBalanceP = kBalanceP.getDouble(0);
    Constants.DriveConstants.kBalanceI = kBalanceI.getDouble(0);
    Constants.DriveConstants.kBalanceD = kBalanceD.getDouble(0);

    turnPid.setP(kTurnP.getDouble(0));
    turnPid.setI(kTurnI.getDouble(0));
    turnPid.setD(kTurnD.getDouble(0));

    // drive.setMaxVoltage(kMaxVoltage.getDouble(0));
    drive.setMaxSpeed(kMaxSpeed.getDouble(0));

    Pose2d pose = drive.getPoseWithVisionMeasurements();

    tick += 1;
    if (tick % 20 == 0) {
      Pose2d pose = drive.getPose();
      System.out.println("Angle: " + pose.getRotation().getDegrees() + " deg. X: " + Units.metersToInches(pose.getX()) + ", Y: " + Units.metersToInches(pose.getY()));
    }

  }

  Command auton;
  Command turnToAngleCommand = null;

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
        new TurnToAngle(360, drive));

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
        new TurnToAngle(180, drive));

    Command followRectangleOdom = new SequentialCommandGroup(
        new MoveDistance(Units.inchesToMeters(20), drive),

        new TurnToAngle(90, drive),
        new MoveDistance(Units.inchesToMeters(10), drive),

        new TurnToAngle(180, drive),
        new MoveDistance(Units.inchesToMeters(20), drive),

        new TurnToAngle(270, drive),
        new MoveDistance(Units.inchesToMeters(10), drive),

        new TurnToAngle(0, drive));

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
    // // new TurnToAngle(-2.2239612403854787, drive).withTimeout(3.5),
    // new MoveDistance(Units.metersToInches(1.051658639992556), drive)
    // );
    // outOfZone.schedule();

    var straightBalancingSequence = new SequentialCommandGroup(
        new MoveDistance(10.517001824211251, drive).withTimeout(3.5),
        new MoveDistance(-8.1151787916318057, drive)
            .withPitchExitThreshold(6)
            .withTimeout(3.5),
        new Balance(drive));

    var meterTest = new SequentialCommandGroup(
        new MoveDistance(1, drive));

    var exitCommunity = new SequentialCommandGroup(
        new MoveDistance(3.5, drive));

    Command jerkBack = new MoveDistance(-Units.inchesToMeters(6), drive)
        .withCustomGains(50, 0, 0)
        .withMaxVoltage(0.5)
        .withTimeout(1)
        .andThen(() -> drive.killMotors())
        .andThen(new MoveDistance(Units.inchesToMeters(6), drive))
        .andThen(() -> drive.killMotors());

    SequentialCommandGroup flipAndPushBackCone = new SequentialCommandGroup(
        new SetArmAndExtender(armSubSys, 50,
            armSubSys.getExtenderPosition() / Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE)
            .withTimeout(2),
        new SetArmAndExtender(armSubSys, 0,
            armSubSys.getExtenderPosition() / Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE)
            .withTimeout(2))
        .andThen(() -> drive.killMotors());

    // SequentialCommandGroup flipConeClimbAndBalance = new SequentialCommandGroup(
    // flipAndPushBackCone,
    // new MoveDistance(2, drive)
    // .withPitchExitThreshold(10)
    // .withTimeout(5),
    // new Balance(drive).withTimeout(9)
    // ).andThen(() -> drive.holdPosition());

    SequentialCommandGroup scoreCubeHighGoal = new SequentialCommandGroup(
      new SetArm(armSubSys, ScoringConstants.HIGH_CUBE_NODE_ARM_ANGLE)
        .withTimeout(0.5),
      new SetExtender(armSubSys, ScoringConstants.HIGH_CUBE_NODE_EXTENDER_ROTATIONS)
        .withTimeout(0.5),
      new Lambda(() -> intake.set(-0.2))
        .withTimeout(0.3)
        .andThen(() -> intake.set(0)),
      new SetExtender(armSubSys, 0)
        .withTimeout(0.5),
      new SetArm(armSubSys, 0)
        .withTimeout(0.5)
    );

    SequentialCommandGroup flipConeAndExitCommunityAndBackUpAndBalance = new SequentialCommandGroup(
        flipAndPushBackCone,
        new MoveDistance(2.4, drive)
            .withTimeout(5),
        new MoveDistance(-2, drive)
            .withPitchExitThreshold(10)
            .withTimeout(5),
        new Balance(drive).withTimeout(9)).andThen(() -> drive.holdPosition());

    SequentialCommandGroup flipCone = new SequentialCommandGroup(
      new SetArm(armSubSys, 50),
      new SetArm(armSubSys, 0)
    );

    // SequentialCommandGroup scoreCubeClimbAndExit = new SequentialCommandGroup(
    //   scoreCubeHighGoal,
    //   new MoveDistance(-4.5, drive)
    //     .withPitchExitThreshold(10)
    //     .withTimeout(5),
    //   new Balance(drive).withTimeout(9)
    // ).andThen(() -> drive.holdPosition());

    SequentialCommandGroup scoreCubeClimbAndExit = new SequentialCommandGroup(
      scoreCubeHighGoal,
      // new MoveDistance(-2.7, drive)
      //   .withTimeout(5),
      new MoveDistance(-3.3, drive)
        .withPitchExitThreshold(10)
        .withTimeout(5),
      new Balance(drive)
    ).andThen(() -> drive.holdPosition());

    // auton = new Balance(drive).andThen(() -> drive.holdPosition());
    // auton = climbAndExit;
    auton = scoreCubeClimbAndExit;
    auton.schedule();

    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxAutonVoltage;
    drive.setMaxVoltage(Constants.DriveConstants.kMaxVoltage);
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
    if (currentMoveToPoseCommand != null) {
      currentMoveToPoseCommand.cancel();
    }
    drive.differentialDrive(0, 0);
    drive.setIdleMode(IdleMode.kCoast);
    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxDriveVoltage;
    if (auton != null)
      auton.cancel(); // Cancel if auton didn't terminate properly
  }

  MjpegServer driverServer;
  PhotonCameraWrapper pCam = new PhotonCameraWrapper();

  double teleopTargetAngle = 0;
  boolean teleopTargetingAngle = false;
  double lastPov = -1;


  @Override
  public void teleopInit() {
    // drive.disablePIDControl();
    drive.setIdleMode(IdleMode.kCoast);
    // drive.setControlMode(ControlMode.VOLTAGE);
    Constants.DriveConstants.kMaxVoltage = Constants.DriveConstants.kMaxDriveVoltage;
    drive.setMaxVoltage(Constants.DriveConstants.kMaxVoltage);
    teleopTargetAngle = drive.getHeading();
    armSubSys.reset();
    // lastPov = joystick.getPOV();
    compressor.disable();
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
  double scaleFactor = 1;

  Command scheduledCommand = null;

  boolean turnIsAbsolute = false;

  int intakeDirection = 0;

  private double velocityCurve(double x, double deadzone, double r, double z, double m) {
    double mag = Utils.deadzone(x, deadzone);
    double value = (Math.pow(mag - r, 5) * z) + m;
    return Math.copySign(value, x);
  }

  private double turnCurve(double x, double deadzone, double scale, double factor) {
    double value = x*scale;
    return Utils.deadzone(Math.copySign(value, x), deadzone);
  }
  //private double turnCurve(double x, double deadzone, double scale, double factor) {
  //  double mag = Math.max(Math.abs(x) - deadzone, 0) * 1 / (1 - deadzone);
  //  double value = scale*Math.pow(mag, factor);
  //  return Math.copySign(value, x);
  //}
  DynamicCommand presetCommand = new DynamicCommand();

  @Override
  public void teleopPeriodic() {
    Pose2d pose = drive.getPoseWithVisionMeasurements();

    // double throttle = 1-(joystick.getThrottle() + 1)/2;
    
    double jx = driveController.getLeftX();
    double jy = -driveController.getRightY();

    double rValue = r.getValue();
    double zValue = z.getValue();
    double mValue = m.getValue();
    double speedDeadzoneValue = speedDeadzone.getValue();
    double fineScaleValue = fineScale.getValue();
    double turnScaleValue = turnScale.getValue();
    double turnDeadzoneValue = turnDeadzone.getValue();
    
    double speed = velocityCurve(jy, speedDeadzoneValue, rValue, zValue, mValue);
    
    double turn;
    double turnFactor = 5;
    double fineTurnFactor = 7;
    if (Math.abs(speed) < turnIsLinearThreshold.getValue()) {
      // TODO: Move to function
      turn = Utils.deadzone(jx, turnDeadzoneValue);
    } else {
      if (driveController.getLeftTriggerAxis() < 0.5) {
        turn = turnCurve(jx, turnDeadzoneValue, turnScaleValue, turnFactor);
      } else {
        System.out.println("Fine Mode Enabled");
        turn = turnCurve(jx, turnDeadzoneValue, fineScaleValue, fineTurnFactor);
      }
    }
    
    if (turnIsAbsolute) {
      teleopTargetAngle = teleopTargetAngle + 5 * turn;
      //nt pov = joystick.getPOV();
      //if (pov != -1) {
        //teleopTargetAngle = (double)pov;
      //}
      turn = turnPid.calculate(drive.getHeading(), teleopTargetAngle);
      //System.out.println("Absolute Turning: current: " + drive.getHeading() + " deg; target: " + teleopTargetAngle + " deg");
    }

    if (driveControllerButtons.getXButtonPressed()) {
      ControlMode newControlMode =
        drive.getControlMode() == ControlMode.VOLTAGE 
          ? ControlMode.VELOCITY 
          : ControlMode.VOLTAGE;
      System.out.println("Robot: Switching drive mode from " + drive.getControlMode().name() + " to " + newControlMode.name());
      drive.setControlMode(newControlMode);
    }
    if(driveController.getStartButton())
        armSubSys.calibrate();
    if(driveController.getLeftStickButton())
      speed = jx < 1e-6 ? 0 : Math.copySign(0.05, jx);
    if(driveController.getRightStickButton())
      turn = jy < 1e-6 ? 0 : Math.copySign(0.05, jy);
    drive.differentialDrive(speed, turn);

    //if (controller.getXButtonPressed()) {
      //if (scheduledCommand != null && scheduledCommand.isScheduled()) scheduledCommand.cancel();

    //  scheduledCommand = new SequentialCommandGroup(
    //    new TurnToAngle(90, drive),
    //    new MoveDistance(1, drive),
    //    new TurnToAngle(0, drive)
    //  );

    //  scheduledCommand.schedule();
    //}

    // Thumb button on top of joystick
    // if (driveController.getRawButtonPressed(7)) turnIsAbsolute = !turnIsAbsolute;
    
    // TODO: Get presets to work
     if (controller.getPOV() == 0) armSubSys.setPreset(ArmSubsystem.ArmTargetChoice.DOUBLE_SUBSTATION);
     if (controller.getPOV() == 180) armSubSys.setPreset(ArmSubsystem.ArmTargetChoice.STOW);

     if (controller.getPOV() == 90) armSubSys.setPreset(ArmSubsystem.ArmTargetChoice.LEVEL_THREE_PLATFORM);
     if (controller.getPOV() == 270) armSubSys.setPreset(ArmSubsystem.ArmTargetChoice.LEVEL_TWO_PLATFORM);

    //if (mechanismControllerButtons.getXButtonPressed()) {
    //  armSubSys.setPreset(ArmSubsystem.ArmTargetChoice.LEVEL_TWO_POLE);
    //}

    // armSubSys.setTargetMode(target);

    if (controller.getRawButton(6)) {
      // if (balanceCommand.isScheduled())
      //   balanceCommand.cancel();
      // else
      //   balanceCommand.schedule();
    }
    //EXTENDER - 3.6 inches for every 5 rotations of the motr
    //double armSpeed = driveController.getLeftTriggerAxis() < 0.5
    //  ? 1.2
    //  : 0.3;

    if(controller.getRightBumper()) {
      armSubSys.changeExtenderPosition(1.2);
    }
    if(controller.getLeftBumper()) {
      armSubSys.changeExtenderPosition(-1.2);
    }

    double armScaleValue = armScale.getValue();
    if(controller.getAButton())
      armScaleValue = armFineScale.getValue();
    armSubSys.changeArmPosition((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())*armScaleValue);

    // Intake    
    
    // TODO: Figure out expected current empty and w/ object and use
    // properly. Disable motor if current goes above and update dashboard
    // if (intake.getOutputCurrent() < currentLimitClaw.getValue()) {
      if (controller.getYButton()) {
        intakeDirection = 1;
      } else if (controller.getAButton()) {
        intakeDirection = -1;
      } else if (controller.getXButton()) {
        intakeDirection = 0;
      }
      intake.set(intakeDirection*intakeSpeed.getValue());
    // } else {
    //   System.out.println("Robot: Clamping intake current at: " + intake.getOutputCurrent());
    //   intake.set(0);
    // }
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

    if (!lastAButtonStatus && driveController.getAButton()) {
      compressorEnabled = !compressorEnabled;
      if (compressorEnabled) {
        System.out.print("COMPRESSOR IS NOW ON\n");
      } else {
        System.out.print("COMPRESSOR IS NOW OFF\n");
      }
    }



    // PISTON
    // if (!lastBButtonStatus && controller.getBButtonPressed()) {
    //   pistonForward = !pistonForward;
    //   if (pistonForward) {
    //     System.out.println("CLOSING PISTON");
    //     leftSolenoid.set(Value.kReverse);
    //     rightSolenoid.set(Value.kForward);
    //   } else {
    //     System.out.println("OPENING PISTON");
    //     leftSolenoid.set(Value.kForward);
    //     rightSolenoid.set(Value.kReverse);
    //   }
    // }

    lastAButtonStatus = driveController.getAButton();
    lastBButtonStatus = controller.getBButton();

  //   if (driveController.getRightStickButtonPressed()) {
  //     if (drive.isUsingVelocity()) {
  //       drive.disablePIDControl();
  //       System.out.println("Robot: Using voltage control");
  //     } else {
  //       drive.enableVelocityControl();
  //       System.out.println("Robot: Using velocity control");
  //     }
  //   }
  }


  Pose2d poseAhead;

  @Override
  public void testInit() {
    armSubSys.setIdleMode(IdleMode.kCoast);
    drive.setIdleMode(IdleMode.kCoast);
  }

  boolean needsToFaceFront = false;

  double lastSetAngle = 0;

  @Override
  public void testPeriodic() {
    // double throttle = 1 - (joystick.getThrottle() + 1) / 2;
    double x = 0; // joystick.getX();
    double y = 0; // -joystick.getY(); // Forward is negative on joystick

    /*

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
    */
  }
}
