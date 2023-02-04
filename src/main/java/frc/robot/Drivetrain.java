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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveTrainConstants;
import org.photonvision.EstimatedRobotPose;
import com.revrobotics.CANSparkMax;

public class Drivetrain {
  // private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  // private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON);

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Constants.DriveTrainConstants.kTrackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator =
    new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final PhotonCameraWrapper pcw;

  private final CANSparkMax[] leftMotors;
  private final CANSparkMax[] rightMotors;

  private CANSparkMax[] motorsFromIds(int[] canIds) {
    CANSparkMax[] motors = new CANSparkMax[canIds.length];
    for (int i = 0; i < canIds.length; i++)
      motors[i] = new CANSparkMax(canIds[i], MotorType.kBrushless);
    return motors;
  }

  public Drivetrain(int[] leftIds, int[] rightIds) {
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
    right.setInverted(true);

    leftEncoder.setPositionConversionFactor(DriveTrainConstants.distancePerPulse);
    rightEncoder.setPositionConversionFactor(DriveTrainConstants.distancePerPulse);

    gyro.reset();
  }

  public void differentialDrive(double speed, double turn) {
    // Positive turn turns right, negative turns left
    left.set(speed + turn);
    right.set(speed - turn);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    poseEstimator.update(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result =
            pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        poseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  public WPI_Pigeon2 getGyro() {
    return gyro;
  }

  public PhotonCameraWrapper getPcw() {
      return pcw;
  }

  public double getPosition() {
    // return (leftEncoder.getPosition() - rightEncoder.getPosition())/2;
    return leftEncoder.getPosition();
  }
}
