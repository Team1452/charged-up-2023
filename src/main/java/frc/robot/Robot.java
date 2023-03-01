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

import org.eclipse.jetty.util.MathUtils;
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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(1, -1, 0);
  private final SlewRateLimiter armSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);

  private final SlewRateLimiter driveSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);
  private final SlewRateLimiter turnSpeedLimiter = new SlewRateLimiter(0.05, -0.05, 0);

  private final AnalogInput pressureSensor = new AnalogInput(0);


  // private final DriveSubsystem drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);
  private final DriveSubsystem drive = new DriveSubsystem(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_RIGHT);


  private final Joystick joystick = new Joystick(0);

  private int tick = 0;

  private final CANSparkMax arm = new CANSparkMax(RobotMap.MOTOR_ARM, MotorType.kBrushless);
  private final CANSparkMax extender = new CANSparkMax(RobotMap.EXTENDER, MotorType.kBrushless);

  double armPosition;
  double extenderPosition;

  private final RelativeEncoder armEncoder = arm.getEncoder();
  private final RelativeEncoder extenderEncoder = extender.getEncoder();

  private SparkMaxPIDController armPID;
  private SparkMaxPIDController extenderPID;

  @Override
  public void robotInit() {
    extender.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ExtenderConstants.MAX_EXTENDER_POSITION);
    extender.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ExtenderConstants.MIN_EXTENDER_POSITION);

    arm.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.MAX_ROTATION_ROT);
    arm.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.MIN_ROTATION_ROT);

    compressor.disable();
    arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extender.setIdleMode(CANSparkMax.IdleMode.kBrake);

    extenderEncoder.setPosition(0);
    armEncoder.setPosition(0);
    arm.setSmartCurrentLimit(10);
    extender.setSmartCurrentLimit(10);
    armPID = arm.getPIDController();
    armPosition = armEncoder.getPosition();

    extenderPID = extender.getPIDController();
    extenderPosition =  extenderEncoder.getPosition();

    armPID.setP(0.05);
    armPID.setI(0.000);
    armPID.setD(0.000);
    armPID.setOutputRange(-1, 1);
    armPID.setIZone(0);
    armPID.setFF(0);

    extenderPID.setP(0.01);
    extenderPID.setI(0.00001);
    extenderPID.setD(0);
    extenderPID.setOutputRange(-1, 1);
    extenderPID.setIZone(0);
    extenderPID.setFF(0);
  }

  @Override
  public void robotPeriodic() {
    tick += 1;
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousExit() {
    System.out.println("Setting idle mode");
  }

  @Override
  public void teleopPeriodic() {
    var speed = Math.pow(-controller.getRightY(), 3);
    var rot = Math.pow(controller.getRightX(), 3);
    // No flip for actual robot
    drive.differentialDrive(speed, rot); // Flip CW/CCW
    //TODO: Temp disabled for later joystick implementation


 //EXTENDER //3.6 inches for every 5 rotations of the motr
//This is reversed, positive is backwards
      if(controller.getLeftBumper())
        extenderPosition += 0.5;
      if(controller.getRightBumper())
        extenderPosition -= 0.5;

    extenderPosition = Math.max(0, Math.min(extenderPosition, 15));

    extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);

    //ARM
    armPosition += controller.getLeftY()*0.1;

    //if (controller.getLeftBumperPressed()) {
    //  armPosition += 5;
    //}
    //if (controller.getLeftBumperPressed()) {
    //  armPosition -= 5;
    //}
    armPID.setReference(armPosition, CANSparkMax.ControlType.kPosition);

    System.out.println("Arm Encoder: " + armEncoder.getPosition());
    System.out.println("Extender Encoder: " + extenderEncoder.getPosition());

    final double armScaleConstant = (Constants.ArmConstants.MAX_ROTATION_ROT-Constants.ArmConstants.MIN_ROTATION_ROT);
    final double armScaleRad = (Constants.ArmConstants.MAX_ROTATION_RAD-Constants.ArmConstants.MIN_ROTATION_RAD)/armScaleConstant;
    //This currently uses the current extender length and then controls for position but below I have code to get to angle and then control for length
    final double currentExtenderLength = Constants.ExtenderConstants.MIN_ARM_LENGTH
          + armEncoder.getPosition() * Constants.ExtenderConstants.METERS_PER_ROTATION;
    final double armHeight = Math.sin(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;




    //System.out.println("arm Height" + armHeight);
    //System.out.println("arm Angle Degrees" + Units.radiansToDegrees(armScaleRad*armEncoder.getPosition()));
    //System.out.println("Arm Height Near Level Two Pole",
    // Math.abs(armHeight - Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT) < 0.1);
    //SmartDashboard.putBoolean("Arm Height Near Level Three Pole",
    // Math.abs(armHeight - Constants.FieldConstants.LEVEL_THREE_POLE_HEIGHT) < 0.1);
    //SmartDashboard.putNumber("Extender Length Meters" , currentExtenderLength);


    if (controller.getXButtonPressed())
      extenderEncoder.setPosition(0);

    if (controller.getBButtonPressed())
      armEncoder.setPosition(0);

    //ARM

    // Run scheduled commands
    CommandScheduler.getInstance().run();
  }


  Pose2d poseAhead;

  double kP = 0.1; 
  double kI = 1e-4;
  double kD = 1; 
  double kIz = 0; 
  double kFF = 0; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;

  double SkP = 0.1; 
  double SkI = 1e-4;
  double SkD = 1; 
  double SkIz = 0; 
  double SkFF = 0; 
  double SkMaxOutput = 1; 
  double SkMinOutput = -1;
  
  @Override
  public void testInit() {
    armPID.setP(kP);
    armPID.setI(kI);
    armPID.setD(kD);
    armPID.setIZone(kIz);
    armPID.setFF(kFF);
    armPID.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Arm P Gain", kP);
    SmartDashboard.putNumber("Arm I Gain", kI);
    SmartDashboard.putNumber("Arm D Gain", kD);
    SmartDashboard.putNumber("Arm I Zone", kIz);
    SmartDashboard.putNumber("Arm Feed Forward", kFF);
    SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    SmartDashboard.putNumber("Arm Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Set Rotations", 0);

    SmartDashboard.putNumber("Extender P Gain", SkP);
    SmartDashboard.putNumber("Extender I Gain", SkI);
    SmartDashboard.putNumber("Extender D Gain", SkD);
    SmartDashboard.putNumber("Extender I Zone", SkIz);
    SmartDashboard.putNumber("Extender Feed Forward", SkFF);
    SmartDashboard.putNumber("Extender Max Output", SkMaxOutput);
    SmartDashboard.putNumber("Extender Min Output", SkMinOutput);
    SmartDashboard.putNumber("Extender Set Rotations", 0);
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;

  boolean controlMode = true;
  double pressure = 0;
  boolean pistonForward2 = false;

  @Override
  public void testPeriodic() {
    double p = SmartDashboard.getNumber("Arm P Gain", 0);
    double i = SmartDashboard.getNumber("Arm I Gain", 0);
    double d = SmartDashboard.getNumber("Arm D Gain", 0);
    double iz = SmartDashboard.getNumber("Arm I Zone", 0);
    double ff = SmartDashboard.getNumber("Arm Feed Forward", 0);
    double max = SmartDashboard.getNumber("Arm Max Output", 0);
    double min = SmartDashboard.getNumber("Arm Min Output", 0);
    double rotations = SmartDashboard.getNumber("Arm Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { armPID.setP(p); kP = p; }
    if((i != kI)) { armPID.setI(i); kI = i; }
    if((d != kD)) { armPID.setD(d); kD = d; }
    if((iz != kIz)) { armPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { armPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      armPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    
    armPID.setReference(rotations, CANSparkMax.ControlType.kPosition);

    //extender   
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", extenderEncoder.getPosition());

    double Sp = SmartDashboard.getNumber("Extender P Gain", 0);
    double Si = SmartDashboard.getNumber("Extender I Gain", 0);
    double Sd = SmartDashboard.getNumber("Extender D Gain", 0);
    double Siz = SmartDashboard.getNumber("Extender I Zone", 0);
    double Sff = SmartDashboard.getNumber("Extender Feed Forward", 0);
    double Smax = SmartDashboard.getNumber("Extender Max Output", 0);
    double Smin = SmartDashboard.getNumber("Extender Min Output", 0);
    double Srotations = SmartDashboard.getNumber("Extender Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((Sp != SkP)) { extenderPID.setP(Sp); SkP = Sp; }
    if((Si != SkI)) { extenderPID.setI(Si); SkI = Si; }
    if((Sd != SkD)) { extenderPID.setD(Sd); SkD = Sd; }
    if((Siz != SkIz)) { extenderPID.setIZone(Siz); SkIz = Siz; }
    if((Sff != SkFF)) { extenderPID.setFF(Sff); SkFF = Sff; }
    if((Smax != SkMaxOutput) || (Smin != SkMinOutput)) { 
      extenderPID.setOutputRange(Smin, Smax); 
      SkMinOutput = Smin; SkMaxOutput = Smax; 
    }

    extenderPID.setReference(Srotations, CANSparkMax.ControlType.kPosition);
  }
}
