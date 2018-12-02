package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive   = null;
    public DcMotor  backRightDrive  = null;

    public DcMotor armBase1 = null;
    public DcMotor armBase2 = null;

    public CRServo intake = null;
    public CRServo winch;
    //public Servo    clawLiftRight    = null;
    //public Servo    clawLiftLeft    = null;
    public ColorSensor color;

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

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left");//Sets the name you have to use for the phone config
        frontRightDrive = hwMap.get(DcMotor.class, "front_right");//Sets the name you have to use for the phone config
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left");//Sets the name you have to use for the phone config
        backRightDrive = hwMap.get(DcMotor.class, "back_right");
	
	winch = hwMap.crservo.get("winch");
	
        // armBase1 = hwMap.get(DcMotor.class, "arm_base_1");
        // armBase2 = hwMap.get(DcMotor.class, "arm_base_2");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // armBase1.setDirection(DcMotor.Direction.FORWARD);
        // armBase2.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        DcMotor[] motors = new DcMotor[] {frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive/*,
            armBase1, armBase2*/};

        for(DcMotor motor: motors) {
            motor.setPower(0); //set all motors to zero power
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set all motors to run with encoders
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set motor behavior to halt when no buttons are pressed
        }
	
        // intake = hwMap.crservo.get("intake");

        //clawLiftRight  = hwMap.get(Servo.class, "claw_lift_right");//Sets the name you have to use for the phone config

        //clawLiftLeft  = hwMap.get(Servo.class, "claw_lift_left");//Sets the name you have to use for the phone config

        // color = hwMap.colorSensor.get("color_sensor");
    }
}
