// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  //private final CANSparkMax arm = new CANSparkMax(18, MotorType.kBrushed);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID[0], RobotMap.SOLENOID[1]);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID2[0], RobotMap.SOLENOID2[1]);
  // private final Solenoid singleSolenoid = new Solenoid(
    // PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID[0]);
  // private final Solenoid singleSolenoid2 = new Solenoid(
    // PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID[1]);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  BangBangController pressureController = new BangBangController();

  @Override
  public void robotInit() {
    compressor.disable();
    //pressureController.setTolerance(5);
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
    leftSolenoid.set(Value.kOff); // Off by default
    rightSolenoid.set(Value.kOff); // Off by default
  }

  boolean pistonForward = false;
  boolean pistonForward2 = false;
  boolean piston3Forward = false;
  boolean compressorEnabled = false;
  double pressure = 0;
  @Override
  public void testPeriodic() {
    pressure = compressor.getPressure(); 
    System.out.println(pressure);
    //arm.set(-Math.pow(controller.getLeftY(), 3));

    if (controller.getRightBumperPressed()) {
      pistonForward = !pistonForward;
      System.out.println("Enabling solenoid: " + pistonForward);
      if (pistonForward)
        rightSolenoid.set(Value.kForward);
      else
        rightSolenoid.set(Value.kReverse);
    }
    if (controller.getLeftBumperPressed()) {
      pistonForward2 = !pistonForward2;
      System.out.println("Enabling solenoid: " + pistonForward2);
      if (pistonForward2)
        leftSolenoid.set(Value.kForward);
      else
        leftSolenoid.set(Value.kReverse);
    }
    // if (controller.getYButton()) {
      // piston2Forward = !piston2Forward;
      // System.out.println("Enabling solenoid: " + piston2Forward);
        // singleSolenoid.set(piston2Forward);
    // }
    // if (controller.getAButton()) {
      // piston3Forward = !piston3Forward;
      // System.out.println("Enabling solenoid: " + piston3Forward);
        // singleSolenoid2.set(piston3Forward);
    // }
     if (compressorEnabled){
        pressureController.calculate(pressure, 50);
        if(pressureController.atSetpoint()){
          compressor.enableDigital();
        }else{
          compressor.disable();
        }
     } else{
        compressor.disable();
    }
    if (controller.getBButtonPressed()) {
      compressorEnabled = !compressorEnabled;
  }
}
}
