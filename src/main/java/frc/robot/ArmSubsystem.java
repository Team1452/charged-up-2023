package frc.robot;
import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.FieldConstants;

public class ArmSubsystem {

    String[] armTargetChoices = {"level two pole", "level three pole", "level three platform", "level two platform", "manual control"};
    String chosenArmTarget = "Manual Control";

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

        if(chosenArmTarget == "manual control"){
            extenderPosition += extenderScaleConstant*percentChange*0.01;
        }

    }

    public void changeArmPosition(double percentChange){
        armHeight = Math.sin(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;

        if(chosenArmTarget == "manual control"){
            armPosition += armScaleConstant*percentChange*0.01;
        }

    }
    public double getArmHeight(){
        return armHeight;
    }
    
    public void update(){
        switch(chosenArmTarget){
            case "level two pole"       : armPosition = Constants.ArmExPos.LEVEL_TWO_POLE_ARM_ANGLE;        
            extenderPosition = Constants.ArmExPos.LEVEL_TWO_POLE_ARM_LENGTH; 
            case "level three pole"     : armPosition = Constants.ArmExPos.LEVEL_THREE_POLE_ARM_ANGLE;      
            extenderPosition = Constants.ArmExPos.LEVEL_THREE_POLE_ARM_LENGTH;
            case "level three platform" : armPosition = Constants.ArmExPos.LEVEL_THREE_PLATFORM_ARM_ANGLE;  
            extenderPosition = Constants.ArmExPos.LEVEL_THREE_PLATFORM_ARM_LENGTH;
            case "level two platform"   : armPosition = Constants.ArmExPos.LEVEL_TWO_PLATFORM_ARM_ANGLE;    
            extenderPosition = Constants.ArmExPos.LEVEL_TWO_PLATFORM_ARM_LENGTH;
            default : ;
        }

        extenderPosition = Math.max(Constants.ExtenderConstants.MIN_EXTENDER_POSITION,
        Math.min(extenderPosition, Constants.ExtenderConstants.MAX_EXTENDER_POSITION));
        extenderPID.setReference(extenderPosition, CANSparkMax.ControlType.kPosition);
        armPosition = Math.max(Constants.ArmConstants.MIN_ROTATION_ROT, Math.min(armPosition, Constants.ArmConstants.MAX_ROTATION_ROT));
        armPID.setReference(armPosition, CANSparkMax.ControlType.kPosition);
    }
}
