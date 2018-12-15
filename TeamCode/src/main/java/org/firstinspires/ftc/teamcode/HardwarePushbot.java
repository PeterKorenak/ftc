package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * This defines the hardware that you are going to use for your robot
 */
public class HardwarePushbot {
    /* Public OpMode members. */
    //DRIVE
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;


    //PRIMARY ARM
    public DcMotor priArm_Right      = null;
    public DcMotor priArm_Left       = null;

    //SECONDARY ARM
    public DcMotor secArm_Right      = null;
    public DcMotor secArm_Left       = null;

    //INTAKE
    public Servo intake_Tilt         = null;
    public Servo intake_Door         = null;
    public CRServo intake_Tumbler    = null;
    public CRServo winch = null;
    public ColorSensor color;

    //MARKER SERVO
    public Servo marker_drop = null;

    //SENSORS


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //COMPONENT INIT//

        //PRIMARY ARM
        // priArm_Right = hwMap.get(DcMotor.class, "pri_arm_right");                          //Sets the name you have to use for the phone config
        // priArm_Left = hwMap.get(DcMotor.class, "pri_arm_left");                            //Sets the name you have to use for the phone config
        // uncomment these once we've got 2 expansion hubs working
        // armBase1 = hwMap.get(DcMotor.class, "arm_base_1");
        // armBase2 = hwMap.get(DcMotor.class, "arm_base_2");

        //SECONDARY ARM
        // secArm_Right = hwMap.get(DcMotor.class, "sec_arm_right");                          //Sets the name you have to use for the phone config
        // secArm_Left = hwMap.get(DcMotor.class, "sec_arm_left");                            //Sets the name you have to use for the phone config

        //DRIVE
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left");                         //Sets the name you have to use for the phone config
        frontRightDrive = hwMap.get(DcMotor.class, "front_right");                        //Sets the name you have to use for the phone config
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left");                           //Sets the name you have to use for the phone config
        backRightDrive = hwMap.get(DcMotor.class, "back_right");                          //Sets the name you have to use for the phone config

        //INTAKE
        // intake_Tilt = hwMap.servo.get("intake_tilt");                                               //Sets intake tilt motor config
        // intake_Tumbler = hwMap.crservo.get("intake_tumbler");                                       //Sets intake tumbler config
        // intake_Door = hwMap.servo.get("intake_door");                                               //Sets intake Door config

        // WINCH
        winch = hwMap.crservo.get("winch");

        //SENSORS
        color = hwMap.colorSensor.get("color_sensor");                                              //Sets color sensor config

        //SERVO TO DROP MARKER IN DEPOT
        marker_drop = hwMap.servo.get("marker");
        //DIRECTION INIT//\

        //PRIMARY ARM
        // priArm_Right.setDirection(DcMotor.Direction.FORWARD);
        // priArm_Left.setDirection(DcMotor.Direction.REVERSE);

        //SECONDARY ARM
        // secArm_Right.setDirection(DcMotor.Direction.FORWARD);
        // secArm_Left.setDirection(DcMotor.Direction.REVERSE);

        //DRIVE
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);                                     // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);                                    // Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);                                      // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //MOTOR BRAKE: sets all mot power to zero
        DcMotor[] motors = new DcMotor[] {frontLeftDrive, frontRightDrive,backLeftDrive,
                backRightDrive/*, priArm_Right, priArm_Left, secArm_Left, secArm_Right*/};

        for(DcMotor motor: motors) {
            motor.setPower(0); //set all motors to zero power
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set all motors to run with encoders
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set motor behavior to halt when no buttons are pressed
        }
    }
}
