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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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


  private final DriveSubsystem drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, RobotMap.TEST_MOTOR_RIGHT);


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
  private final ArmSubsystem armSubSys = new ArmSubsystem(RobotMap.MOTOR_ARM, RobotMap.EXTENDER);

  private final SendableChooser<String> armChooser = new SendableChooser<>();
  @Override
  public void robotInit() {
    arm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    extender.setIdleMode(CANSparkMax.IdleMode.kCoast);
    arm.restoreFactoryDefaults();
    extender.restoreFactoryDefaults();

    compressor.disable();

    extenderEncoder.setPosition(Constants.ExtenderConstants.MIN_EXTENDER_POSITION);
    armEncoder.setPosition(Constants.ArmConstants.MIN_ROTATION_ROT);
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


    //extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ExtenderConstants.MAX_EXTENDER_POSITION);
    //extender.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0f);
  }

  @Override
  public void robotPeriodic() {
    tick += 1;
  }

  @Override
  public void autonomousInit() {

    arm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    extender.setIdleMode(CANSparkMax.IdleMode.kCoast);

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
  }

  @Override
  public void autonomousExit() {
    arm.setIdleMode(CANSparkMax.IdleMode.kCoast); //TODO: VERY IMPORTANT DISABLE THIS FOR COMPETITION
    extender.setIdleMode(CANSparkMax.IdleMode.kCoast);
    System.out.println("Setting idle mode");
    drive.setIdleMode(IdleMode.kCoast);
  }
  String[] armTargetChoices = {"Level Two Pole", "Level Three Pole", "Level Three Platform", "Level Two Platform", "Manual Control"};
  String chosenArmTarget = "Manual Control";
  @Override
  public void teleopPeriodic() {
    //var speed = Math.pow(-joystick.getY(), 3);
    //var rot = Math.pow(joystick.getX(), 3);
    // No flip for actual robot
    //drive.differentialDrive(speed, rot); // Flip CW/CCW
    //TODO: Temp disabled for later joystick implementation


 //EXTENDER //3.6 inches for every 5 rotations of the motr
      final double extenderScaleConstant = (Constants.ExtenderConstants.MAX_EXTENDER_POSITION - Constants.ExtenderConstants.MIN_EXTENDER_POSITION); 

      if(controller.getLeftBumper())
        armSubSys.changeArmPosition(5);
      if(controller.getRightBumper())
        armSubSys.changeArmPosition(-5);

    extenderPosition = Math.max(Constants.ExtenderConstants.MIN_EXTENDER_POSITION,
     Math.min(extenderPosition, Constants.ExtenderConstants.MAX_EXTENDER_POSITION));

    extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);


    //ARM

    final double armScaleConstant = (Constants.ArmConstants.MAX_ROTATION_ROT-Constants.ArmConstants.MIN_ROTATION_ROT);
    final double armScaleRad = (Constants.ArmConstants.MAX_ROTATION_RAD-Constants.ArmConstants.MIN_ROTATION_RAD)/armScaleConstant;

    //This currently uses the current extender length and then controls for position but below I have code to get to angle and then control for length
    final double currentExtenderLength = Constants.ExtenderConstants.MIN_ARM_LENGTH
          + armEncoder.getPosition() * Constants.ExtenderConstants.METERS_PER_ROTATION;
    final double armHeight = Math.sin(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;

    armPosition += armScaleConstant*0.01*controller.getLeftY();

    armPosition = Math.max(Constants.ArmConstants.MIN_ROTATION_ROT, Math.min(armPosition, Constants.ArmConstants.MAX_ROTATION_ROT));
    armPID.setReference(armPosition, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("arm Height", armHeight);
    SmartDashboard.putNumber("arm Angle Degrees", Units.radiansToDegrees(armScaleRad*armEncoder.getPosition()));
    SmartDashboard.putBoolean("Arm Height Near Level Two Pole",
     Math.abs(armHeight - Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT) < 0.1);
    SmartDashboard.putBoolean("Arm Height Near Level Three Pole",
     Math.abs(armHeight - Constants.FieldConstants.LEVEL_THREE_POLE_HEIGHT) < 0.1);
    SmartDashboard.putNumber("Extender Length Meters" , currentExtenderLength);
    SmartDashboard.putNumber("Extender Encoder" , extenderEncoder.getPosition());
    SmartDashboard.putNumber("Arm Encoder" , armEncoder.getPosition());
    System.out.print("Arm Encoder: " + armEncoder.getPosition());
    SmartDashboard.putNumber("arm Height", armHeight);
    if(controller.getYButtonPressed()){ //control to position of Level Three Pole
      
    }

    if (controller.getXButtonPressed())
      extenderEncoder.setPosition(0);

    if (controller.getBButtonPressed())
      armEncoder.setPosition(0);
    if (controller.getAButtonPressed()) {
      System.out.println("Trigger pressed, turning to 180 deg");
      new TurnToAngle(180, drive).schedule();
    }

    var gyro = drive.getGyro();
    //System.out.println("Yaw: " + gyro.getYaw() + ", pitch: " + gyro.getPitch() + ", roll: " + gyro.getRoll());

    // if (controller.getAButtonPressed()) {
    //   solenoid.set(Value.kForward);
    // } else if (controller.getAButtonReleased()) {
    //   solenoid.set(Value.kReverse);
    // }

    //ARM

    // Run scheduled commands
    CommandScheduler.getInstance().run();
  }


  Pose2d poseAhead;

  @Override
  public void testInit() {
    arm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    extender.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
    double speed = Math.pow(-controller.getRightY(), 3.0);
    double turn = Math.pow(controller.getRightX(), 3.0);

    // System.out.println("Joystick y: " + joystick.getY() + " ; joystick x: " + joystick.getX());
    // double joystickY = Math.signum(joystick.getY()) * Math.max(0, Math.abs(joystick.getY()) - 0.1);
    // double joystickX = Math.signum(joystick.getX()) * Math.max(0, Math.abs(joystick.getX()) - 0.1);
    // speed = driveSpeedLimiter.calculate(-joystickY);
    // turn = turnSpeedLimiter.calculate(joystickX);
    // System.out.println("Speed: " + speed + "; turn: " + turn);

    drive.differentialDrive(speed, -turn);

    // EXTENDER
    if(controller.getLeftBumper()){
      extenderPosition=extenderPosition+0.01;
    }
    if(controller.getRightBumper()){
      extenderPosition=extenderPosition-0.01;
    }
    extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);
    System.out.println("Extender position: " + extenderPosition);

    // ARM
    /*
    if (controller.getLeftBumper()) {
      // Turn counterclockwise
      armAngle += 0.3;
    }

    if (controller.getRightBumper()) {
      // Turn clockwise
      armAngle -= 0.3;
    }*/

    ///armPID.setReference(armAngle, CANSparkMax.ControlType.kPosition);

    // COMPRESSOR
    if (compressorEnabled) {
      // Calculate pressure from analog input
      // (from REV analog sensor datasheet)
      double vOut = pressureSensor.getAverageVoltage();
      double pressure = 250 * (vOut / Constants.PneumaticConstants.ANALOG_VCC) - 25;

      pressureController.calculate(pressure, Constants.PneumaticConstants.TARGET_PRESSURE);
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
