// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.StreamReadCapability;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  private final XboxController controller = new XboxController(0);
  private final CANSparkMax arm = new CANSparkMax(RobotMap.MOTOR_ARM, MotorType.kBrushless);
  private final CANSparkMax extender = new CANSparkMax(RobotMap.MOTOR_EXTEND, MotorType.kBrushless);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final SlewRateLimiter extenderSlewLimiter = new SlewRateLimiter(0.5,-0.5,0);
  private SparkMaxPIDController armPID;
  RelativeEncoder armEncoder = arm.getEncoder();
  RelativeEncoder extenderEncoder = arm.getEncoder();
  double pos;
  @Override
  public void robotInit() {
    compressor.disable();
    arm.restoreFactoryDefaults();
    arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extender.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //armEncoder = arm.getEncoder();
    //extenderEncoder = arm.getEncoder();
    extenderEncoder.setPosition(0);
    armEncoder.setPosition(0);
    armPID = arm.getPIDController();
    pos = armEncoder.getPosition();
    armPID.setP(0.1);
    armPID.setI(0.0001);
    armPID.setD(0.001);
    armPID.setOutputRange(-1, 1);
    armPID.setIZone(0);
    armPID.setFF(0);
    // sets absolute encoder limits for arm
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)Constants.ArmConstants.MIN_ROTATION);
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)Constants.ArmConstants.MAX_ROTATION);
    //armPID.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {

  }
  final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.cameraName);



  @Override
  public void teleopPeriodic() {
    //var result = camera.getLatestResult();
    //PhotonTrackedTarget target = result.getBestTarget();
    //double armHeight = Math.tan()
  }

  @Override
  public void testInit() {
  }

  boolean pistonForward = false;
  boolean compressorEnabled = false;
  
  boolean controlMode = true; 
  @Override
  public void testPeriodic() { 
    if(controlMode){
    
    if(controller.getBButtonPressed()){
      pos=pos+5;
    }
    if(controller.getXButtonPressed()){
      pos=pos-5;
    }
  }else{
    pos = controller.getLeftY();
    
  }
    if(controller.getBackButtonPressed()){
      //controlMode = !controlMode;
      System.out.println("extender Encoder values are " + extenderEncoder.getPosition());
    }
    //System.out.println("arm Encoder values are " + armEncoder.getPosition());
    //System.out.println("arm Vel values are " + armEncoder.getVelocity());
    //System.out.println("reference is: " + pos);
    armPID.setReference(pos, CANSparkMax.ControlType.kPosition);

    if(controller.getAButtonPressed()){
    }
    if(controller.getYButtonPressed()){
      double armLength = Constants.ExtenderConstants.MIN_ARM_LENGTH + armEncoder.getPosition()*Constants.ExtenderConstants.METERS_PER_ROTATION;
      double angle = Math.atan((Constants.FieldConstants.LEVEL_TWO_POLE_HEIGHT-Constants.ArmConstants.ARM_HEIGHT)/ armLength);
      System.out.println("armLength: " + armLength);
      System.out.println("arm angle: " + angle);
      pos = Constants.ArmConstants.ARM_GEARING*angle;
    }
//extenderSlewLimiter.calculate(0.2)
//Then here we get robot pose to object and from that decide how far arm needs to extend
    if(controller.getLeftBumper()){
      extender.set(0.2);
    }else if(controller.getRightBumper()){
      extender.set(-0.2);
    }else{
      extender.set(0);
    }

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
