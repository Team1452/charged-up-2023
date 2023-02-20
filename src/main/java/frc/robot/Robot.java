// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.BangBangController;
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final CANSparkMax extender = new CANSparkMax(RobotMap.MOTOR_EXTEND, MotorType.kBrushless);

  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);
  private final SlewRateLimiter armSlewLimiter = new SlewRateLimiter(0.2, -0.2, 0);

  private final AnalogInput pressureSensor = new AnalogInput(0);

  private SparkMaxPIDController armPID;
  private SparkMaxPIDController extenderPID;

  private final Drivetrain drivetrain = new Drivetrain(RobotMap.MOTOR_LEFT, RobotMap.MOTOR_RIGHT);

  private final RelativeEncoder armEncoder = arm.getEncoder();
  private final RelativeEncoder extenderEncoder = arm.getEncoder();

  double armAngle;
  double extenderPosition;

  @Override
  public void robotInit() {
    compressor.disable();
    arm.restoreFactoryDefaults();
    arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extender.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // armEncoder = arm.getEncoder();
    // extenderEncoder = arm.getEncoder();
    extenderEncoder.setPosition(0);
    armEncoder.setPosition(0);
    armPID = arm.getPIDController();
    armAngle = armEncoder.getPosition();

    extenderEncoder.setPosition(0);
    extenderPID = extender.getPIDController();
    extenderPosition = extenderEncoder.getPosition();

    armPID.setP(0.1);
    armPID.setI(0.0001);
    armPID.setD(0.001);
    armPID.setOutputRange(-1, 1);
    armPID.setIZone(0);
    armPID.setFF(0);

    // sets absolute encoder limits for arm
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ArmConstants.MIN_ROTATION);
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.ArmConstants.MAX_ROTATION);
    // armPID.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {

  }

  final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.cameraName);

  @Override
  public void teleopPeriodic() {
    // var result = camera.getLatestResult();
    // PhotonTrackedTarget target = result.getBestTarget();
    // double armHeight = Math.tan()
  }

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
    // DRIVE
    // Controller forward is negative
    double speed = Math.pow(-controller.getLeftY(), 3.0);
    double turn = Math.pow(controller.getRightX(), 3.0);

    drivetrain.differentialDrive(speed, turn);

    // EXTENDER
    extenderPosition += extenderSlewLimiter.calculate(controller.getLeftTriggerAxis())
        - extenderSlewLimiter.calculate(controller.getRightTriggerAxis());
    extenderPID.setReference(extenderPosition, ControlType.kPosition);

    // ARM
    if (controller.getLeftBumperPressed()) {
      // Turn counterclockwise
      armAngle += 1;
    }

    if (controller.getRightBumperPressed()) {
      // Turn clockwise
      armAngle -= 1;
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
