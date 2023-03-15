package frc.robot;
import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
public class ArmSubsystem {

    public enum ArmTargetChoice {
        MANUAL_CONTROL,
        STOW,
        LEVEL_TWO_POLE,
        LEVEL_TWO_PLATFORM,
        LEVEL_THREE_POLE,
        LEVEL_THREE_PLATFORM,
        DOUBLE_SUBSTATION,
    }

    private ArmTargetChoice targetChoice = ArmTargetChoice.MANUAL_CONTROL;
    private CANSparkMax arm;
    private CANSparkMax extender; 

    double armPosition = 0;
    public double extenderPosition = 0;

    double oldArmPosition;
    double oldExtenderPosition;

    private final RelativeEncoder armEncoder;
    private final RelativeEncoder extenderEncoder;

    private SparkMaxPIDController armPID;
    private SparkMaxPIDController extenderPID;
    private final double armScaleConstant = (Constants.ArmConstants.MAX_ROTATION_ROT-Constants.ArmConstants.MIN_ROTATION_ROT);
    private final double armScaleRad = (Constants.ArmConstants.RANGE_RAD)/armScaleConstant - Constants.ArmConstants.START_ANGLE;
    private double currentExtenderLength;
    private double armY;
    private double armX;
    final double extenderScaleConstant = (Constants.ExtenderConstants.MAX_EXTENDER_ROTATIONS - Constants.ExtenderConstants.MIN_EXTENDER_ROTATIONS); 

    private double getArmAngleRadians() {
        return armPosition/(Constants.ArmConstants.MAX_ROTATION_ROT - Constants.ArmConstants.MIN_ROTATION_ROT) * Constants.ArmConstants.RANGE_RAD + Constants.ArmConstants.START_ANGLE;
    }

    public ArmSubsystem(int armID, int extenderID){
        arm = new CANSparkMax(armID, MotorType.kBrushless);
        extender = new CANSparkMax(extenderID, MotorType.kBrushless);
        arm.restoreFactoryDefaults();
        extender.restoreFactoryDefaults();
        arm.setIdleMode(CANSparkMax.IdleMode.kCoast);
        extender.setIdleMode(CANSparkMax.IdleMode.kCoast);
        arm.restoreFactoryDefaults();
        extender.restoreFactoryDefaults();

        armEncoder = arm.getEncoder();
        extenderEncoder = extender.getEncoder();
        armEncoder.setPosition(0);
        extenderEncoder.setPosition(0);
        extenderEncoder.setPosition(Constants.ExtenderConstants.MIN_EXTENDER_ROTATIONS);
        armEncoder.setPosition(Constants.ArmConstants.MIN_ROTATION_ROT);
        armPID = arm.getPIDController();
        extenderPID = extender.getPIDController();
        //Set gains in robotInit
        extenderPosition = extenderEncoder.getPosition();
        armPosition = armEncoder.getPosition();

        oldExtenderPosition = extenderPosition;
        oldArmPosition = armPosition;

        armPID.setP(0.1);
        armPID.setI(0.000);
        armPID.setD(0.00);
        armPID.setOutputRange(-1, 1);
        armPID.setIZone(0);
        armPID.setFF(0);

        extenderPID.setP(0.1);
        extenderPID.setI(0);
        extenderPID.setD(0);
        extenderPID.setOutputRange(-1, 1);
        extenderPID.setIZone(0);
        extenderPID.setFF(0);

        // extenderEncoder.setPositionConversionFactor(1.0/Constants.ExtenderConstants.EXTENDER_ROTATION_RANGE);
        // armEncoder.setPositionConversionFactor(1.0/Constants.ArmConstants.ARM_ROTATION_RANGE_ROT);
    }

    private void updateSavedPositions() {
        oldExtenderPosition = extenderPosition;
        oldArmPosition = armPosition;
    }

    private void restoreOldPositions() {
        extenderPosition = oldExtenderPosition;
        armPosition = oldArmPosition;
    }

    private boolean isInLegalPosition() {
        double armAngle = getArmAngleRadians();
        boolean isLegal = (Math.cos(armAngle) * currentExtenderLength) + Units.inchesToMeters(32/2) <= Units.inchesToMeters(48 - 6);
        if (!isLegal) System.out.println("ArmSubsystem: Position is illegal");
        return isLegal;
    }

    public void reset() {
        armPosition = 0;
        extenderPosition = 0;
        armEncoder.setPosition(0);
        extenderEncoder.setPosition(0);
    }

    public void changeExtenderPosition(double percentChange){
        updateSavedPositions();
        if(percentChange > 0)
            targetChoice = ArmTargetChoice.MANUAL_CONTROL;
        if (targetChoice == ArmTargetChoice.MANUAL_CONTROL){
            extenderPosition += extenderScaleConstant*percentChange*0.01;
        }
    }

    public void changeArmPosition(double percentChange){
        updateSavedPositions();
        if(percentChange > 0)
            targetChoice = ArmTargetChoice.MANUAL_CONTROL;
        if (targetChoice == ArmTargetChoice.MANUAL_CONTROL) 
            armPosition += armScaleConstant*percentChange*0.01;
    }

    public void setIdleMode(CANSparkMax.IdleMode mode){
        arm.setIdleMode(mode);
        extender.setIdleMode(mode);
    }

    public void setTargetMode(ArmTargetChoice t){
        targetChoice = t;
    }

    public ArmTargetChoice update() {
        currentExtenderLength = Constants.ExtenderConstants.MIN_ARM_LENGTH
        + extenderEncoder.getPosition() * Constants.ExtenderConstants.METERS_PER_ROTATION;

        ArmTargetChoice out = ArmTargetChoice.MANUAL_CONTROL;
        switch(targetChoice){
            case STOW:
                armPosition = Constants.ArmConstants.MIN_ROTATION_RAD;
                extenderPosition = Constants.ExtenderConstants.MIN_EXTENDER_ROTATIONS; 
                out = ArmTargetChoice.STOW;
                break;
            case LEVEL_TWO_POLE:
                armPosition = Constants.ScoringConstants.LOW_CONE_NODE_ARM_ANGLE;        
                extenderPosition = Constants.ScoringConstants.LOW_CONE_NODE_EXTENDER_ROTATIONS; 
                out = ArmTargetChoice.LEVEL_TWO_POLE;
                break;
            case LEVEL_THREE_POLE:
                armPosition = Constants.ScoringConstants.HIGH_CONE_NODE_ARM_ANGLE;      
                extenderPosition = Constants.ScoringConstants.HIGH_CONE_NODE_EXTENDER_ROTATIONS;
                out = ArmTargetChoice.LEVEL_THREE_POLE;
                break;
            case LEVEL_THREE_PLATFORM:
                armPosition = Constants.ScoringConstants.HIGH_CUBE_NODE_ARM_ANGLE;  
                extenderPosition = Constants.ScoringConstants.HIGH_CUBE_NODE_EXTENDER_ROTATIONS;
                out = ArmTargetChoice.LEVEL_THREE_PLATFORM;
                break;
            case LEVEL_TWO_PLATFORM:
                armPosition = Constants.ScoringConstants.LOW_CUBE_NODE_ARM_ANGLE;    
                extenderPosition = Constants.ScoringConstants.LOW_CUBE_NODE_EXTENDER_ROTATIONS;
                out = ArmTargetChoice.LEVEL_TWO_PLATFORM;
                break;
            case DOUBLE_SUBSTATION:
                armPosition = Constants.ScoringConstants.DRIVER_STATION_ARM_ANGLE;  
                extenderPosition = Constants.ScoringConstants.DRIVER_STATION_EXTENDER_ROTATIONS;
                out = ArmTargetChoice.DOUBLE_SUBSTATION;
                break;
            default:
                break;
        }

        armY = Math.sin(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;
        armX = Math.cos(armEncoder.getPosition() * armScaleRad) * currentExtenderLength;

        // if we're at the arm position then we switch back to manual control
        if(Math.abs(armEncoder.getPosition()-armPosition)<0.5 && Math.abs(extenderEncoder.getPosition()-extenderPosition)<0.5 ){
            out = ArmTargetChoice.MANUAL_CONTROL;
        }


        if (targetChoice == ArmTargetChoice.MANUAL_CONTROL) {
            System.out.printf("ArmSubsystem: Manual control: Setting extender to %.3f, at %.3f. Setting arm to %.3f, is %.3f\n", extenderPosition, extenderEncoder.getPosition(), armPosition, armEncoder.getPosition());
            setExtenderPosition(extenderPosition);
            setArmPosition(armPosition);
        }

        return out;
    }

    public void setExtenderPosition(double extenderPosition) {
        extenderPosition = MathUtil.clamp(extenderPosition, Constants.ExtenderConstants.MIN_EXTENDER_ROTATIONS, Constants.ExtenderConstants.MAX_EXTENDER_ROTATIONS);
        extenderPID.setReference(-extenderPosition, CANSparkMax.ControlType.kPosition);
    }

    public void setArmPosition(double armPosition) {
        armPosition = MathUtil.clamp(armPosition, Constants.ArmConstants.MIN_ROTATION_ROT, Constants.ArmConstants.MAX_ROTATION_ROT);
        armPID.setReference(armPosition, CANSparkMax.ControlType.kPosition);
    }
    
    public double getArmHeight(){
        return armY;
    }
    public double getArmDistance(){
        return armX;
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
