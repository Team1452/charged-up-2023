// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.photonvision.EstimatedRobotPose;
import com.revrobotics.CANSparkMax;

public class DriveSubsystem extends SubsystemBase {
  // private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.MOTOR_LEFT,
  // CANSparkMaxLowLevel.MotorType.kBrushless);
  // private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.MOTOR_RIGHT,
  // CANSparkMaxLowLevel.MotorType.kBrushless);

  public final MotorControllerGroup left;
  public final MotorControllerGroup right;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final boolean leftInverted;
  private final boolean rightInverted;

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.DriveConstants.kTrackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
      gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final DifferentialDrivePoseEstimator poseEstimatorWithVision = new DifferentialDrivePoseEstimator(kinematics,
      gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final PhotonCameraWrapper pcw;

  private final CANSparkMax[] leftMotors;
  private final CANSparkMax[] rightMotors;

  private double maxVoltage = 0.35;
  private double maxSpeed = Constants.DriveConstants.kMaxSpeed; // m/s

  private boolean usingVelocity = false;

  public boolean isUsingVelocity() {
      return usingVelocity;
  }

  public void setMaxVoltage(double maxVoltage) {
    this.maxVoltage = maxVoltage;
  }

  public void killMotors() {
    disablePIDControl();

    for (CANSparkMax motor : leftMotors) {
      motor.set(0);
    }

    for (CANSparkMax motor : rightMotors) {
      motor.set(0);
    }
  }

  /**
   * Set gains for each controller's PID for velocity control.
   * Note that as long as gains are non-zero controllers cannot be
   * controlled directly
   */
  public void enableVelocityControl() {
    usingVelocity = true;


    for (CANSparkMax motor : leftMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(Constants.DriveConstants.kVelocityP);
      controller.setI(Constants.DriveConstants.kVelocityI);
      controller.setD(Constants.DriveConstants.kVelocityD);
    }

    for (CANSparkMax motor : rightMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(Constants.DriveConstants.kVelocityP);
      controller.setI(Constants.DriveConstants.kVelocityI);
      controller.setD(Constants.DriveConstants.kVelocityD);
    }
  }

  public void disablePIDControl() {
    usingVelocity = false;

    for (CANSparkMax motor : leftMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(0);
      controller.setI(0);
      controller.setD(0);
    }
    for (CANSparkMax motor : rightMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(0);
      controller.setI(0);
      controller.setD(0);
    }
  }

  public void holdPosition() {
    for (CANSparkMax motor : leftMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(0.01);
      controller.setI(0);
      controller.setD(0);
      controller.setReference(motor.getEncoder().getPosition(), ControlType.kPosition);
    }
    for (CANSparkMax motor : rightMotors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setP(0.01);
      controller.setI(0);
      controller.setD(0);
      controller.setReference(motor.getEncoder().getPosition(), ControlType.kPosition);
    }
  }

  private void setVelocities(CANSparkMax[] motors, double velocity) {
    for (CANSparkMax motor : motors) {
      SparkMaxPIDController controller = motor.getPIDController();
      controller.setReference(velocity, ControlType.kVelocity);
    }
  }

  public void setIdleMode(IdleMode mode) {
    for (CANSparkMax motor : leftMotors)
      motor.setIdleMode(mode);
    for (CANSparkMax motor : rightMotors)
      motor.setIdleMode(mode);
  }

  private CANSparkMax[] motorsFromIds(int[] canIds) {
    CANSparkMax[] motors = new CANSparkMax[canIds.length];
    for (int i = 0; i < canIds.length; i++) {
      motors[i] = new CANSparkMax(canIds[i], MotorType.kBrushless);
      // motors[i].setIdleMode(IdleMode.kBrake); // Set idle mode to brake
      motors[i].setIdleMode(IdleMode.kCoast); // Set idle mode to brake
    }
    return motors;
  }

  public DriveSubsystem(int[] leftIds, boolean leftInverted, int[] rightIds, boolean rightInverted) {
    pcw = new PhotonCameraWrapper();
    leftMotors = motorsFromIds(leftIds);
    rightMotors = motorsFromIds(rightIds);

    left = new MotorControllerGroup(leftMotors);
    right = new MotorControllerGroup(rightMotors);

    leftEncoder = leftMotors[0].getEncoder();
    rightEncoder = rightMotors[0].getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftEncoder.setPositionConversionFactor(DriveConstants.kDistancePerPulse);
    rightEncoder.setPositionConversionFactor(DriveConstants.kDistancePerPulse);

    this.leftInverted = leftInverted;
    this.rightInverted = rightInverted;

    left.setInverted(leftInverted);
    right.setInverted(rightInverted);

    gyro.reset();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  private void setWithLimit(MotorController controller, double value) {
    double limitedValue = Math.copySign(Math.min(Math.abs(value), maxVoltage), value);
    // System.out.printf("Setting to %.3f; ", limitedValue);
    controller.set(limitedValue);
  }

  private void differentialDriveVoltage(double speed, double turn) {
    // Positive turn turns right, negative turns left
    // System.out.printf("DriveSubsystem: Max voltage is " + maxVoltage + "; ");
    setWithLimit(left, speed + turn);
    setWithLimit(right, speed - turn);
    // System.out.println();
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  public void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
  }

  private void differentialDriveVelocity(double speed, double turn) {
    // Positive turn turns right, negative turns left
    double leftVelocity = speed + turn;
    double rightVelocity = speed - turn;

    double max = Math.max(Math.abs(leftVelocity), Math.abs(rightVelocity));

    if (max > maxSpeed) {
      leftVelocity /= max;
      rightVelocity /= max;
    }

    setVelocities(leftMotors, leftVelocity);
    setVelocities(rightMotors, rightVelocity);
  }

  public void differentialDrive(double speed, double turn) {
    if (isUsingVelocity()) {
      differentialDriveVelocity(speed, turn);
    } else {
      differentialDriveVoltage(speed, turn);
    }
  }

  double getLeftPosition() {
    double position = leftEncoder.getPosition();
    if (leftInverted)
      return -position;
    else
      return position;
  }

  double getRightPosition() {
    double position = rightEncoder.getPosition();
    if (rightInverted)
      return -position;
    else
      return position;
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    // Left encoder is inverted
    poseEstimator.update(
        gyro.getRotation2d(), getLeftPosition(), getRightPosition());
    poseEstimatorWithVision.update(
        gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      poseEstimatorWithVision.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  public Pose2d getPoseWithVisionMeasurements() {
    return poseEstimatorWithVision.getEstimatedPosition();
  }

  public Pose2d getPoseFromOdometry() {
    return poseEstimator.getEstimatedPosition();
  }

  public WPI_Pigeon2 getGyro() {
    return gyro;
  }

  public PhotonCameraWrapper getPcw() {
    return pcw;
  }

  public double getPosition() {
    return (getLeftPosition() + getRightPosition()) / 2;
  }

  public void resetPosition(Pose2d pose) {
    poseEstimatorWithVision.resetPosition(gyro.getRotation2d(), getLeftPosition(), getRightPosition(), pose);
  }

  public void resetPositionOdometry() {
    gyro.reset();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
