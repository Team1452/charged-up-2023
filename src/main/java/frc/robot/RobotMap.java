package frc.robot;

public class RobotMap {
    // Global flag for whether to use testbed values
    public static boolean USING_TESTBED = false;
    

    public static int[] TEST_MOTOR_LEFT = {22};
    public static int[] TEST_MOTOR_RIGHT = {16};
    public static boolean TEST_MOTOR_LEFT_INVERTED = false;
    public static boolean TEST_MOTOR_RIGHT_INVERTED = true;

    public static int[] MOTOR_LEFT = {14};
    public static int[] MOTOR_RIGHT = {3};
    public static boolean MOTOR_LEFT_INVERTED = false;
    public static boolean MOTOR_RIGHT_INVERTED = true;
    public static int MOTOR_ARM = 12;
    public static int MOTOR_EXTEND = 15;
    public static int PIGEON = 10;

    public static int[] SOLENOID_1 = {2, 5};
    public static int[] SOLENOID_2 = {3, 4};
    public static int ANGLE_CONVERSION = -5;
}
