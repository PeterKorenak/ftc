/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Comp TeleOp", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_Iterative extends OpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .9;
    static final double     TURN_SPEED              = .9;
    double servoPos = 0.0;
    boolean doorClosed = true;
    int thing = 30;
    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot = new org.firstinspires.ftc.teamcode.HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //double          clawBarrierOffset  = -1 ; //Each servo you want to move seperately
    //double          clawRotationOffset = 1;   //has to have its own variable offset
    //double          clawLiftOffset = -1;
    //final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double contPower;
    double winchElevation = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    @Override
    public void loop() {
        /*
        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot
        double left, right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.left_stick_y;

        drive = gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;*/


        // WINCH
//        if (gamepad1.b) {
//            robot.winch.setPower(1.0);
//        } else if (gamepad1.b) { // yes this makes no sense but it sorta works
//            robot.winch.setPower(-1.0);
//        } else {
//	    robot.winch.setPower(0.0);
//	}

        // ARM UP
        if(gamepad1.a) {
            int position1 = robot.priArm_Left.getCurrentPosition();
            int position2 = robot.priArm_Right.getCurrentPosition();

            robot.priArm_Left.setTargetPosition(position1 + 10);
            robot.priArm_Left.setPower(.3);

            robot.priArm_Right.setTargetPosition(position2 + 10);
            robot.priArm_Right.setPower(.3);
        // ARM DOWN
        } else if(gamepad1.b) {
            int position3 = robot.priArm_Left.getCurrentPosition();
            int position4 = robot.priArm_Right.getCurrentPosition();

            robot.priArm_Left.setTargetPosition(position3 - 10);
            robot.priArm_Left.setPower(-.3);

            robot.priArm_Right.setTargetPosition(position4 - 10);
            robot.priArm_Right.setPower(-.3);

        } else {
            robot.priArm_Left.setPower(0);
            robot.priArm_Right.setPower(0);
        }


        // SECONDARY ARM UP
        if(gamepad1.y) {
            int position5 = robot.secArm_Left.getCurrentPosition();
            int position6 = robot.secArm_Right.getCurrentPosition();

            robot.secArm_Left.setTargetPosition(position5 + 10);
            robot.secArm_Left.setPower(.45);

            robot.secArm_Right.setTargetPosition(position6 - 10);
            robot.secArm_Right.setPower(-0.45);
            // ARM DOWN
        } else if(gamepad1.x) {
            int position7 = robot.secArm_Left.getCurrentPosition();
            int position8 = robot.secArm_Right.getCurrentPosition();

            robot.secArm_Left.setTargetPosition(position7 - 10);
            robot.secArm_Left.setPower(-0.45);

            robot.secArm_Right.setTargetPosition(position8 + 10);
            robot.secArm_Right.setPower(.45);
        } else {
            robot.secArm_Left.setPower(0);
            robot.secArm_Right.setPower(0);
        }
        // STRAFING
        // notice that the front left & back right drives need to be a little
        // faster to account for hardware stuff
        if(gamepad1.dpad_left) {
            robot.frontLeftDrive.setPower(-.7);
            robot.frontRightDrive.setPower(.7);
            robot.backLeftDrive.setPower(.7);
            robot.backRightDrive.setPower(-.7);
        } else if(gamepad1.dpad_right) {
            robot.frontLeftDrive.setPower(.7);
            robot.frontRightDrive.setPower(-.7);
            robot.backLeftDrive.setPower(-.7);
            robot.backRightDrive.setPower(.7);
        } /*else if(gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            double r = Math.hypot(gamepad1.right_stick_x, (-(gamepad1.right_stick_y)));
            double robotAngle = Math.atan2((-(gamepad1.right_stick_y)), gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeftDrive.setPower(v1/13);
            robot.frontRightDrive.setPower(v2/13);
            robot.backLeftDrive.setPower(v3/13);
            robot.backRightDrive.setPower(v4/13);}*/
        else if(gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0)
        {
            if(gamepad1.right_stick_x > .5)
            {
                robot.frontRightDrive.setPower(-.09);
                robot.backRightDrive.setPower(-.09);
                robot.frontLeftDrive.setPower(.09);
                robot.backLeftDrive.setPower(.09);
            }
            else if (gamepad1.right_stick_x < -.5)
        {
            robot.frontRightDrive.setPower(.09);
            robot.backRightDrive.setPower(.09);
            robot.frontLeftDrive.setPower(-.09);
            robot.backLeftDrive.setPower(-.09);
        }
            else if(gamepad1.right_stick_y != 0)
            {
                double p = .3 * gamepad1.right_stick_y;
                robot.frontRightDrive.setPower(p);
                robot.backRightDrive.setPower(p);
                robot.frontLeftDrive.setPower(p);
                robot.backLeftDrive.setPower(p);
            }
        }

         /*else if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            double l = Math.hypot(gamepad1.left_stick_x, (-(gamepad1.left_stick_y)));
            double robotAngle = Math.atan2((-(gamepad1.left_stick_y)), gamepad1.left_stick_x) - Math.PI / 4;
            double leftX = gamepad1.left_stick_x;
            final double v1 = l * Math.cos(robotAngle) + leftX;
            final double v2 = l * Math.sin(robotAngle) - leftX;
            final double v3 = l * Math.sin(robotAngle) + leftX;
            final double v4 = l * Math.cos(robotAngle) - leftX;

            robot.frontLeftDrive.setPower(v1);
            robot.frontRightDrive.setPower(v2);
            robot.backLeftDrive.setPower(v3);
            robot.backRightDrive.setPower(v4);
        }*/
        else if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            if (gamepad1.left_stick_x > .5) {
                robot.frontRightDrive.setPower(-.7);
                robot.backRightDrive.setPower(-.7);
                robot.frontLeftDrive.setPower(.7);
                robot.backLeftDrive.setPower(.7);
            } else if (gamepad1.left_stick_x < -.5) {
                robot.frontRightDrive.setPower(.7);
                robot.backRightDrive.setPower(.7);
                robot.frontLeftDrive.setPower(-.7);
                robot.backLeftDrive.setPower(-.7);
            } else if (gamepad1.left_stick_y != 0) {
                double p = .8 * gamepad1.left_stick_y;
                robot.frontRightDrive.setPower(p);
                robot.backRightDrive.setPower(p);
                robot.frontLeftDrive.setPower(p);
                robot.backLeftDrive.setPower(p);
            }
        }

        else {
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
        }


        // INTAKE STUFF


        // press back to set lock mode

        if (gamepad1.left_bumper) {
            robot.intake_Tumbler.setPower(.6);
        } else if (gamepad1.right_bumper) {
            robot.intake_Tumbler.setPower(-.6);
        } else robot.intake_Tumbler.setPower(0);

        if(gamepad1.dpad_up) {
            robot.intake_Door.setPosition(0);
        } else if(gamepad1.dpad_down) {
            robot.intake_Door.setPosition(180);
        }

        if(gamepad1.right_trigger != 0)
            robot.intake_tilt.setPosition(.49);
        else if(gamepad1.left_trigger != 0)
            robot.intake_tilt.setPosition(.71);
        //else robot.intake_tilt.setPosition(robot.intake_tilt.getPosition());
    }

    public void encoderDrive(double speed,
                             double fleftInches, double frightInches,
                             double bleftInches, double brightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (fleftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (frightInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.backLeftDrive.getCurrentPosition() + (int) (bleftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.backRightDrive.getCurrentPosition() + (int) (brightInches * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newLeftTarget);
            robot.frontRightDrive.setTargetPosition(newRightTarget);
            robot.backLeftDrive.setTargetPosition(newLeftTarget2);
            robot.backRightDrive.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontRightDrive.getCurrentPosition(),
                        robot.frontLeftDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
