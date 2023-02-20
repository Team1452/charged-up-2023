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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  private final CANSparkMax arm = new CANSparkMax(15, MotorType.kBrushed);
  private final CANSparkMax joint = new CANSparkMax(12, MotorType.kBrushed);
  private final DoubleSolenoid solenoid1 = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_1[0], RobotMap.SOLENOID_1[1]);
  private final DoubleSolenoid solenoid2 = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_2[0], RobotMap.SOLENOID_2[1]);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final AnalogInput pressureSensor = new AnalogInput(0);

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
    solenoid1.set(Value.kOff); // Off by default
    solenoid2.set(Value.kOff);
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;

  @Override
  public void testPeriodic() {
    double armSpeed = -Math.pow(controller.getLeftY(), 3);
    arm.set(armSpeed);

    double jointSpeed = -Math.pow(controller.getRightY(), 3);
    joint.set(jointSpeed);

    if (controller.getAButtonPressed()) {
      pistonForward = !pistonForward;
      System.out.println("Enabling solenoid: " + pistonForward);
      if (pistonForward) {
        solenoid1.set(Value.kReverse);
        solenoid2.set(Value.kForward);
      } else {
        solenoid1.set(Value.kForward);
        solenoid2.set(Value.kReverse);
      }
    }

    if (controller.getBButtonPressed()) {
      compressorEnabled = !compressorEnabled;
      if (compressorEnabled)
        compressor.enableDigital();
      else
        compressor.disable();
    }

    double vout = pressureSensor.getAverageVoltage();
    double vcc = 5; // Could be 3.33/needs to be measured?
    double pressure = 250 * (vout/vcc) - 25;
    System.out.printf("Voltage is %.3f, estimated pressure is %.3f PSI\n", vout, pressure);
  }
}
