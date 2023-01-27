// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain drive = new Drivetrain();
  private final PIDController balancer = new PIDController(0.001, 0, 0);
  private final PhotonCameraWrapper pcw = new PhotonCameraWrapper();

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    var speed = Math.pow(controller.getLeftY(), 3);
    var rot = Math.pow(controller.getRightX(), 3);

    System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    drive.drive(speed, rot);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    // Should technically be .getPitch(), but Pigeon
    // isn't oriented/calibrated correctly
    double pitch = drive.getGyro().getRoll();
    double speed = balancer.calculate(pitch, 0);
    System.out.println("pitch: " + pitch + "; speed: " + speed);
    drive.drive(speed, 0);
  }
}
