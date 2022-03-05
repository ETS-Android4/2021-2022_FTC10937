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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Iterative OpMode")

public class FTC10937TeleOp extends OpMode {
    // Classes runtime + drivetrain + intake + lift + servo
    ElapsedTime runtime = new ElapsedTime();
    driveTrainSetup Drive = new driveTrainSetup();
    intakeSetup Intake = new intakeSetup();
    carouselSetup Carousel = new carouselSetup();
    liftSetup Lift = new liftSetup();
    servoSetup Servo = new servoSetup();

    int servoCount = 0;
    int rightBump = 0;

    @Override
    public void init() {
        // Initialize motors + servos
        Lift.init(hardwareMap);
        Carousel.init(hardwareMap);
        Drive.init(hardwareMap);
        Intake.init(hardwareMap);
        Servo.init(hardwareMap);

        // initialized
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // Reset runtime after driver taps play
        runtime.reset();
    }

    @Override
    // TeleOp code goes here
    public void loop() {
        // Minimum threshold for deadzone on controller
        double minThreshold = 0.2;

        // Drive motors
        // If controller value > threshold then move robot
        if(Math.abs(gamepad1.left_stick_y) > minThreshold || Math.abs(gamepad1.left_stick_x) > minThreshold
        ||Math.abs(gamepad1.right_stick_x) > minThreshold) {
            Drive.right1.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            Drive.left1.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            Drive.right2.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            Drive.left2.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        } else {
            Drive.right1.setPower(0);
            Drive.left1.setPower(0);
            Drive.right2.setPower(0);
            Drive.left2.setPower(0);
        }

        // Intake Motor
        if(gamepad2.a) {
            // Intake in
            Intake.intakeMotor.setPower(1);
        } else if(gamepad2.x) {
            // Intake out
            Intake.intakeMotor.setPower(-1);
        } else {
            Intake.intakeMotor.setPower(0);
        }

        // Lift motor
        // Might change to triggers later for better speed control
        if(gamepad2.dpad_up) {
            // Lift up
            Lift.liftMotor.setPower(.3);
        } else if(gamepad2.dpad_down) {
            // Lift down
            Lift.liftMotor.setPower(-.3);
        } else {
            Lift.liftMotor.setVelocity(0);
        }

        // Carousel motor
        if(gamepad2.dpad_left) {
            // Turn left
            Carousel.carouselMotor.setPower(1);
        } else if(gamepad2.dpad_right) {
            // Turn right
            Carousel.carouselMotor.setPower(-1);
        } else {
            Carousel.carouselMotor.setPower(0);
        }

        // Bucket servo individual control
        // Used especially for nudging elements in
        // Also used to secure ball and duck on their way up
        switch (rightBump) {
            // First rb input = face servo up
            case 0:
                if(gamepad2.right_bumper) {
                    Servo.bucketServo.setPosition(.5);
                    rightBump = 1;
                }
                break;
            // wait for release
            case 1:
                rightBump = (!gamepad2.right_bumper ? 2:1);
            // Second rb input = reset servo position
            case 2:
                if(gamepad2.right_bumper) {
                    Servo.bucketServo.setPosition(0);
                    rightBump = 3;
                }
            // wait for release
            case 3:
                rightBump = (!gamepad2.right_bumper ? 0:3);
            // default = wait for first input
            default:
                rightBump = 0;
                break;
        }

        // Scoring Servos :)
        switch (servoCount) {
            // First y input = extend arm but do not dump element
            case 0:
                if(gamepad2.y) {
                    Servo.rotServo.setPosition(1);
                    Servo.bucketServo.setPosition(1);
                    servoCount = 1;
                }
                break;
            // wait for button release
            case 1:
                if(!gamepad2.y) {
                    servoCount = 2;
                }
                break;
            // Second y input = dump element
            case 2:
                if(gamepad2.y) {
                    Servo.bucketServo.setPosition(0);
                    servoCount = 3;
                }
                break;
            // wait for button release
            case 3:
                if(!gamepad2.y) {
                    servoCount = 4;
                }
                break;
            // Third y input = retract servo arm
            case 4:
                if(gamepad2.y) {
                    Servo.rotServo.setPosition(0);
                    Servo.bucketServo.setPosition(0);
                    servoCount = 5;
                }
                break;
            // wait for release
            case 5:
                if(!gamepad2.y) {
                    servoCount = 0;
                }
                break;
            // default = look for first y input
            default:
                servoCount = 0;
                break;
        }


        telemetry.addData("rpm: ", Drive.left1.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}

// ¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
// ¶¶¶¶¶¶¶¶__________¶¶¶¶¶¶¶¶
// ¶¶¶¶¶_______________¶¶¶¶¶¶
// ¶¶¶____________________¶¶¶
// ¶¶______¶¶______________¶¶
// ¶______¶¶¶¶______________¶
// ¶______¶¶¶¶____¶¶¶¶______¶
// ¶_______¶¶_______________¶
// ¶________________________¶
// ¶_____¶¶__________¶¶_____¶
// ¶¶_____¶¶________¶¶_____¶¶
// ¶¶¶______¶¶¶¶¶¶¶¶¶_____¶¶¶
// ¶¶¶¶¶________________¶¶¶¶¶
// ¶¶¶¶¶¶¶¶___________¶¶¶¶¶¶¶
// ¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
