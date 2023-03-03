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


  private final DriveSubsystem drive = new DriveSubsystem(RobotMap.TEST_MOTOR_LEFT, false, RobotMap.TEST_MOTOR_RIGHT, true);


  private final Joystick joystick = new Joystick(0);

  private int tick = 0;

  private final ArmSubsystem armSubSys = new ArmSubsystem(RobotMap.MOTOR_ARM, RobotMap.MOTOR_EXTEND);
  private int mode = 0;
  private ArmSubsystem.ArmTargetChoice[] targetModes = {
    ArmSubsystem.ArmTargetChoice.MANUAL_CONTROL,
    ArmSubsystem.ArmTargetChoice.LEVEL_THREE_PLATFORM,
     ArmSubsystem.ArmTargetChoice.LEVEL_THREE_POLE,
     ArmSubsystem.ArmTargetChoice.LEVEL_TWO_PLATFORM,
     ArmSubsystem.ArmTargetChoice.LEVEL_TWO_POLE
    };
  @Override
  public void robotInit() {
    compressor.disable();


  }

  @Override
  public void robotPeriodic() {
    tick += 1;
  }

  @Override
  public void autonomousInit() {
    armSubSys.setIdleMode(CANSparkMax.IdleMode.kCoast);

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

    new CenterPhotonVisionTarget(drive).schedule();
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
  }
  @Override
  public void teleopPeriodic() {
 //EXTENDER //3.6 inches for every 5 rotations of the motr
      if(controller.getLeftBumper())
        armSubSys.changeArmPosition(0.5);
      if(controller.getRightBumper())
        armSubSys.changeArmPosition(-0.5);

    if(controller.getRightTriggerAxis()>0.3){
      armSubSys.changeExtenderPosition(controller.getRightTriggerAxis());
    }
    if(controller.getLeftTriggerAxis()>0.3){
      armSubSys.changeExtenderPosition(-controller.getLeftTriggerAxis());
    }
    System.out.println("Extender position: " + armSubSys.getArmEncoder().getPosition());

    System.out.println("Arm Encoder: " + armSubSys.getArmEncoder().getPosition());
    System.out.println("target Mode: " + targetModes[mode]);

    if (controller.getAButtonPressed()) {
      System.out.println("Trigger pressed, turning to 180 deg");
      new TurnToAngle(180, drive).schedule();
    }

    //System.out.println("Yaw: " + gyro.getYaw() + ", pitch: " + gyro.getPitch() + ", roll: " + gyro.getRoll());

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
    armSubSys.setIdleMode(CANSparkMax.IdleMode.kCoast);
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

    drive.differentialDrive(speed, -turn);

    // EXTENDER
    if(controller.getRightTriggerAxis()>0){
      armSubSys.changeExtenderPosition(-controller.getRightTriggerAxis());
    }
    if(controller.getLeftTriggerAxis()>0){
      armSubSys.changeExtenderPosition(-controller.getLeftTriggerAxis());
    }
    System.out.println("Extender position: " + armSubSys.getArmEncoder().getPosition());

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

/*
    SmartDashboard.putNumber("arm Height", armSubSys.getArmHeight());
    SmartDashboard.putNumber("arm Angle Degrees", Units.radiansToDegrees(armScaleRad*armEncoder.getPosition()));
    SmartDashboard.putBoolean("Arm Height Near Level Two Pole",
     Math.abs(armSubSys.getArmHeight() - Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT) < 0.1);
    SmartDashboard.putBoolean("Arm Height Near Level Three Pole",
     Math.abs(armSubSys.getArmHeight() - Constants.FieldConstants.LEVEL_THREE_POLE_HEIGHT) < 0.1);
    SmartDashboard.putNumber("Extender Encoder" , extenderEncoder.getPosition());
    SmartDashboard.putNumber("Arm Encoder" , armEncoder.getPosition());
    SmartDashboard.putNumber("arm Height", armSubSys.getArmHeight());
    SmartDashboard.putNumber("arm Height", armSubSys.getArmHeight());
    SmartDashboard.putNumber("arm Angle Degrees", Units.radiansToDegrees(armScaleRad*armEncoder.getPosition()));
    SmartDashboard.putBoolean("Arm Height Near Level Two Pole",
     Math.abs(armSubSys.getArmHeight() - Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT) < 0.1);
    SmartDashboard.putBoolean("Arm Height Near Level Three Pole",
     Math.abs(armSubSys.getArmHeight() - Constants.FieldConstants.LEVEL_THREE_POLE_HEIGHT) < 0.1);
    SmartDashboard.putNumber("Extender Encoder" , extenderEncoder.getPosition());
    SmartDashboard.putNumber("Arm Encoder" , armEncoder.getPosition());
    SmartDashboard.putNumber("arm Height", armSubSys.getArmHeight());
 */

  }
}
