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
  // private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  // private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON);

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator =
    new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final DifferentialDrivePoseEstimator poseEstimatorWithVision = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final PhotonCameraWrapper pcw;

  private final CANSparkMax[] leftMotors;
  private final CANSparkMax[] rightMotors;

  private double maxVoltage = 0.35;

  public void setMaxVoltage(double maxVoltage) {
      this.maxVoltage = maxVoltage;
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
      motors[i].setIdleMode(IdleMode.kBrake); // Set idle mode to brake 
    }
    return motors;
  }

  public DriveSubsystem(int[] leftIds, int[] rightIds) {
    pcw = new PhotonCameraWrapper();
    leftMotors = motorsFromIds(leftIds);
    rightMotors = motorsFromIds(rightIds);

    left = new MotorControllerGroup(leftMotors);
    right = new MotorControllerGroup(rightMotors);

    leftEncoder = leftMotors[0].getEncoder();
    rightEncoder = rightMotors[0].getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // Invert right motor (positive should be forward, negative backward)
    // left.setInverted(true);
    leftEncoder.setPositionConversionFactor(DriveConstants.kDistancePerPulse);
    rightEncoder.setPositionConversionFactor(DriveConstants.kDistancePerPulse);

    if (DriveConstants.LEFT_MOTOR_INVERTED) {
      left.setInverted(true);
      leftEncoder.setPositionConversionFactor(-left.getPositionConversionFactor())
    }

    if (DriveConstants.RIGHT_MOTOR_INVERTED) {
      right.setInverted(true);
      rightEncoder.setPositionConversionFactor(-rightEncoder.getPositionConversionFactor())
    }

    gyro.reset();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  private void setWithLimit(MotorController controller, double value) {
    double limitedValue = Math.signum(value) * Math.min(Math.abs(value), maxVoltage);
    controller.set(limitedValue);
  }

  public void differentialDrive(double speed, double turn) {
    // Positive turn turns right, negative turns left
    setWithLimit(left, speed + turn);
    setWithLimit(right, speed - turn);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    // Left encoder is inverted
    poseEstimator.update(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    poseEstimatorWithVision.update(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result =
            pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

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
    // Left is inverted
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }

  public void resetPosition(Pose2d pose) {
    poseEstimatorWithVision.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  public void resetPositionOdometry() {
    gyro.reset();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
