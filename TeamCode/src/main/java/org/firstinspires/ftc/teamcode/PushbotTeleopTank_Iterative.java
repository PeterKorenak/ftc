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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
        double backRightPower = drive + strafe - rotate;
        */

        // WINCH
        if (gamepad1.y) {
            robot.winch.setPosition(1.5);
        } else if (gamepad1.y) {
            robot.winch.setPosition(0.5);
        }

        // ARM UP
//        if(gamepad1.a) {
//            int position1 = robot.priArm_Left.getCurrentPosition();
//            int position2 = robot.priArm_Right.getCurrentPosition();
//
//            robot.priArm_Left.setTargetPosition(position1 + 10);
//            robot.priArm_Left.setPower(.3);
//
//            robot.priArm_Right.setTargetPosition(position2 + 10);
//            robot.priArm_Right.setPower(.3);
//        //ARM DOWN
//        } else if(gamepad1.y) {
//            int position3 = robot.priArm_Left.getCurrentPosition();
//            int position4 = robot.priArm_Right.getCurrentPosition();
//
//            robot.priArm_Left.setTargetPosition(position3 - 10);
//            robot.priArm_Left.setPower(-.3);
//
//            robot.priArm_Right.setTargetPosition(position4 - 10);
//            robot.priArm_Right.setPower(-.3);
//
//        } else {
//            robot.priArm_Left.setPower(0);
//            robot.priArm_Right.setPower(0);
//        }

        // STRAFING
        // notice that the front left & back right drives need to be a little
        // faster to account for hardware stuff
        if(gamepad1.dpad_left) {
            robot.frontLeftDrive.setPower(.8);
            robot.frontRightDrive.setPower(-.7);
            robot.backLeftDrive.setPower(-.7);
            robot.backRightDrive.setPower(.8);
        } else if(gamepad1.dpad_right) {
            robot.frontLeftDrive.setPower(-.8);
            robot.frontRightDrive.setPower(.7);
            robot.backLeftDrive.setPower(.7);
            robot.backRightDrive.setPower(-.8);
        } else if(gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeftDrive.setPower(v1);
            robot.frontRightDrive.setPower(v2);
            robot.backLeftDrive.setPower(v3);
            robot.backRightDrive.setPower(v4);
        } else if(gamepad1.right_bumper) {
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
        } else if(gamepad1.left_bumper) {
            robot.frontLeftDrive.setPower(-1);
            robot.frontRightDrive.setPower(1);
            robot.backLeftDrive.setPower(-1);
            robot.backRightDrive.setPower(1);
        } else {
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
        }

        /*
        // INTAKE STUFF
        if(gamepad1.a) {
            contPower = 0;
        }
        else if(gamepad1.b) {
            contPower = 1;
        } else {
            contPower = .45;
        }

        robot.intake.setPower(contPower);
        */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
