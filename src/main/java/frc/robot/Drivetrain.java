// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants.DriveTrainConstants;
import org.photonvision.EstimatedRobotPose;
import com.revrobotics.CANSparkMax;

public class Drivetrain {
  private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON);

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Constants.DriveTrainConstants.kTrackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator =
    new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

  private final PhotonCameraWrapper pcw = new PhotonCameraWrapper();

  public Drivetrain() {
    // Invert right motor (positive should be forward, negative backward)
    rightMotor.setInverted(true);

    leftEncoder.setPositionConversionFactor(DriveTrainConstants.distancePerPulse);
    rightEncoder.setPositionConversionFactor(DriveTrainConstants.distancePerPulse);

    gyro.reset();
  }

  public void drive(double speed, double turn) {
    // Positive turn turns right, negative turns left
    leftMotor.set(speed + turn);
    rightMotor.set(speed - turn);
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
}
