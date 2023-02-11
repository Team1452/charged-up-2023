// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
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
  private final CANSparkMax arm = new CANSparkMax(RobotMap.MOTOR_ARM, MotorType.kBrushless);
  private final CANSparkMax extender = new CANSparkMax(RobotMap.MOTOR_EXTEND, MotorType.kBrushless);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private SparkMaxPIDController armPID;
  private AbsoluteEncoder armEncoder;
  @Override
  public void robotInit() {
    compressor.disable();
    arm.restoreFactoryDefaults();
    armEncoder = arm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // armPID = arm.getPIDController();
    // armPID.setP(0.001);
    // armPID.setI(0.001);
    // armPID.setD(0);
    // armPID.setOutputRange(-1, 1);
    // sets absolute encoder limits for arm
    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, RobotMap.ABSOLUTE_FORWARD_LIMIT);
    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, RobotMap.ABSOLUTE_BACK_LIMIT);
    //armPID.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    //  private final PIDController balancer = new PIDController(0.001, 0, 0);
    // var speed = -Math.pow(controller.getLeftY(), 3);
    // var rot = Math.pow(controller.getRightX(), 3);

    // System.out.println("controller: " + controller.getLeftY() + ", " + controller.getRightX() + "; speed: " + speed + "; rot:" + rot);

    // drive.differentialDrive(speed, rot);

  }

  @Override
  public void testInit() {
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;
  @Override
  public void testPeriodic() {
    arm.set(Math.pow(controller.getLeftY(), 3));
    if(controller.getLeftBumper()){
      extender.set(0.2);
    }else if(controller.getRightBumper()){
      extender.set(-0.2);
    }else{
      extender.set(0);
    }
    System.out.println("arm Encoder values are " + armEncoder.getPosition() );
    //Pneumatics stuff
    // if (controller.getAButtonPressed()) {
    //   pistonForward = !pistonForward;
    //   System.out.println("Enabling solenoid: " + pistonForward);
    //   if (pistonForward)
    //     solenoid.set(Value.kForward);
    //   else
    //     solenoid.set(Value.kReverse);
    // }

    // if (controller.getBButtonPressed()) {
    //   compressorEnabled = !compressorEnabled;
    //   if (compressorEnabled)
    //     compressor.enableDigital();
    //   else
    //     compressor.disable();
    // }
  }
}
