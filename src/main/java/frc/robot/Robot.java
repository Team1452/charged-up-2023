// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain drive = new Drivetrain();
  private final PIDController balancer = new PIDController(0.008, 0.0001, 0);

  private double kP = 0.00006;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000015;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;

  @Override
  public void robotInit() {
    // Configure drivetrain PID parameters
    drive.updateControllers(kP, kI, kDefaultPeriod, kIz, kFF, kMaxOutput, kMinOutput);

    // Display values on GUI
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    kP = SmartDashboard.getNumber("P Gain", 0);
    kI = SmartDashboard.getNumber("I Gain", 0);
    kD = SmartDashboard.getNumber("D Gain", 0);
    kIz = SmartDashboard.getNumber("I Zone", 0);
    kFF = SmartDashboard.getNumber("Feed Forward", 0);
    kMaxOutput = SmartDashboard.getNumber("Max Output", 0);
    kMinOutput = SmartDashboard.getNumber("Min Output", 0);

    drive.updateControllers(kP, kI, kDefaultPeriod, kIz, kFF, kMaxOutput, kMinOutput);

    // Drive
    var speed = -Math.pow(controller.getLeftY(), 3);

    var rot = Math.pow(controller.getLeftX(), 3);

    System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    drive.driveVelocity(speed, rot);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    // Should technically be .getPitch(), but Pigeon
    // isn't oriented/calibrated correctly
    double pitch = drive.getPigeon().getRoll();
    double speed = -balancer.calculate(pitch, 0);
    System.out.println("pitch: " + pitch + "; speed: " + speed);
    drive.driveVoltage(speed, 0);
  }
}
