// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

public class Drivetrain {
  private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final SparkMaxPIDController leftController = leftMotor.getPIDController();
  private final SparkMaxPIDController rightController = rightMotor.getPIDController();

  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.PIGEON);

  private double maxRPM = 5820;

  private void updateControllerGains(SparkMaxPIDController controller, double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setIZone(kIz);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void updateControllers(double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput) {
    updateControllerGains(leftController, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
    updateControllerGains(rightController, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
  }

  public Drivetrain() {
    // Invert right motor (positive should be forward, negative backward)
    rightMotor.setInverted(true);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void driveVelocity(double speed, double turn) {
    // Control velocity through built-in PID 
    double leftRPM = (speed + turn) * maxRPM;
    double rightRPM = (speed - turn) * maxRPM;
    leftController.setReference(leftRPM, ControlType.kVelocity);
    rightController.setReference(rightRPM, ControlType.kVelocity);

    System.out.println("Left controller: setpoint is " + leftRPM + " RPM, currently at " + leftEncoder.getVelocity() + " RPM");
    System.out.println("Right controller: setpoint is " + rightRPM + " RPM, currently at " + rightEncoder.getVelocity() + " RPM");
  }

  public void driveVoltage(double speed, double turn) {
    // Positive turn turns right, negative turns left
    leftMotor.set(speed + turn);
    rightMotor.set(speed - turn);
  }

  public WPI_Pigeon2 getPigeon() {
    return pigeon;
  }
}
