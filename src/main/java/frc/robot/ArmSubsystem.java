package frc.robot;
import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.FieldConstants;
public class ArmSubsystem {

    public enum ArmTargetChoice {
        LEVEL_TWO_POLE,
        LEVEL_TWO_PLATFORM,
        LEVEL_THREE_POLE,
        LEVEL_THREE_PLATFORM,
        MANUAL_CONTROL
    }
    private ArmTargetChoice targetChoice = ArmTargetChoice.MANUAL_CONTROL;
    private CANSparkMax arm;
    private CANSparkMax extender; 

    double armPosition = 0;
    double extenderPosition = 0;

    private final RelativeEncoder armEncoder;
    private final RelativeEncoder extenderEncoder;

    private SparkMaxPIDController armPID;
    private SparkMaxPIDController extenderPID;
    private final double armScaleConstant = (Constants.ArmConstants.MAX_ROTATION_ROT-Constants.ArmConstants.MIN_ROTATION_ROT);
    private final double armScaleRad = (Constants.ArmConstants.MAX_ROTATION_RAD-Constants.ArmConstants.MIN_ROTATION_RAD)/armScaleConstant;
    private double currentExtenderLength;
    private double armHeight;
    final double extenderScaleConstant = (Constants.ExtenderConstants.MAX_EXTENDER_POSITION - Constants.ExtenderConstants.MIN_EXTENDER_POSITION); 

    public ArmSubsystem(int armID, int extenderID){
        arm = new CANSparkMax(armID, MotorType.kBrushless);
        extender = new CANSparkMax(extenderID, MotorType.kBrushless);
        armEncoder = arm.getEncoder();
        extenderEncoder = extender.getEncoder();
        armPID = arm.getPIDController();
        extenderPID = extender.getPIDController();
        //Set gains in robotInit
        extenderPosition =  extenderEncoder.getPosition();
        armPosition = armEncoder.getPosition();
        }

    public void changeExtenderPosition(double percentChange){
        currentExtenderLength = Constants.ExtenderConstants.MIN_ARM_LENGTH
        + armEncoder.getPosition() * Constants.ExtenderConstants.METERS_PER_ROTATION;

        if(targetChoice == ArmTargetChoice.MANUAL_CONTROL){
            extenderPosition += extenderScaleConstant*percentChange*0.01;
        }

    }

    public void changeArmPosition(double percentChange){
        armHeight = Math.sin(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;

        if(targetChoice == ArmTargetChoice.MANUAL_CONTROL){
            armPosition += armScaleConstant*percentChange*0.01;
        }

    }

    public void setTargetMode(ArmTargetChoice t){
        targetChoice = t;
    }
    public void update(){
        switch(targetChoice){
            case LEVEL_TWO_POLE       : armPosition = Constants.ArmExPos.LEVEL_TWO_POLE_ARM_ANGLE;        
            extenderPosition = Constants.ArmExPos.LEVEL_TWO_POLE_ARM_LENGTH; 
            case LEVEL_THREE_POLE     : armPosition = Constants.ArmExPos.LEVEL_THREE_POLE_ARM_ANGLE;      
            extenderPosition = Constants.ArmExPos.LEVEL_THREE_POLE_ARM_LENGTH;
            case LEVEL_THREE_PLATFORM : armPosition = Constants.ArmExPos.LEVEL_THREE_PLATFORM_ARM_ANGLE;  
            extenderPosition = Constants.ArmExPos.LEVEL_THREE_PLATFORM_ARM_LENGTH;
            case LEVEL_TWO_PLATFORM : armPosition = Constants.ArmExPos.LEVEL_TWO_PLATFORM_ARM_ANGLE;    
            extenderPosition = Constants.ArmExPos.LEVEL_TWO_PLATFORM_ARM_LENGTH;
            default : ;
        }

        extenderPosition = Math.max(Constants.ExtenderConstants.MIN_EXTENDER_POSITION,
        Math.min(extenderPosition, Constants.ExtenderConstants.MAX_EXTENDER_POSITION));
        extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);

        armPosition = Math.max(Constants.ArmConstants.MIN_ROTATION_ROT,
         Math.min(armPosition, Constants.ArmConstants.MAX_ROTATION_ROT));
        armPID.setReference(armPosition, CANSparkMax.ControlType.kPosition);
    }
    public double getArmHeight(){
        return armHeight;
    }

    public double getExtenderPosition(){
        return extenderPosition;
    }
    public double getArmPosition(){
        return armPosition;
    }

    public RelativeEncoder getExtenderEncoder(){
        return extenderEncoder;
    }
    public RelativeEncoder getArmEncoder(){
        return armEncoder;
    }
    public SparkMaxPIDController getArmPID(){
        return armPID;
    }
    public SparkMaxPIDController getExtenderPID(){
        return extenderPID;
    }

}
