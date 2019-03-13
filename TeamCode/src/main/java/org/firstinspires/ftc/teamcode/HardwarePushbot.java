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
    public DcMotor  leftDrive;
    public DcMotor  rightDrive;
    public DcMotor  arm;
    //SENSORS
    // public ColorSensor color;

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
    leftDrive  = hwMap.get(DcMotor.class, "left");                         //Sets the name you have to use for the phone config
      rightDrive = hwMap.get(DcMotor.class, "right");                        //Sets the name you have to use for the phone config
        arm  = hwMap.get(DcMotor.class, "arm");


        //DRIVE
        leftDrive.setDirection(DcMotor.Direction.FORWARD);                                     // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);                                    // Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotor.Direction.FORWARD);                                      // Set to REVERSE if using AndyMark motors

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //MOTOR BRAKE: sets all mot power to zero
        DcMotor[] motors = new DcMotor[] {leftDrive, rightDrive,arm};

        for(DcMotor motor: motors) {
            motor.setPower(0); //set all motors to zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set motor behavior to halt when no buttons are pressed
        }
    }
}
