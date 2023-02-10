// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Vector;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  //private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  @Override
  public void robotInit() {
    compressor.disable();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    // var speed = -Math.pow(controller.getLeftY(), 3);
    // var rot = Math.pow(controller.getRightX(), 3);

    // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    // drive.differentialDrive(speed, rot);

  }

  @Override
  public void testInit() {
    solenoid.set(Value.kOff); // Off by default
  }

  boolean pistonForward = false;
  boolean piston2Forward = false;
  boolean compressorEnabled = false;

  @Override
  public void testPeriodic() {
    //arm.set(-Math.pow(controller.getLeftY(), 3));

    if (controller.getRightBumperPressed()) {
      pistonForward = !pistonForward;
      System.out.println("Enabling solenoid: " + pistonForward);
      if (pistonForward)
        solenoid.set(Value.kForward);
      else
        solenoid.set(Value.kReverse);
    }
    if (controller.getLeftBumperPressed()) {
      piston2Forward = !piston2Forward;
      System.out.println("Enabling solenoid: " + piston2Forward);
      if (piston2Forward)
        solenoid.set(Value.kForward);
      else
        solenoid.set(Value.kReverse);
    }
    if (controller.getBButtonPressed()) {
      compressorEnabled = !compressorEnabled;
      if (compressorEnabled)
        compressor.enableDigital();
      else
        compressor.disable();
    }
  }
}
