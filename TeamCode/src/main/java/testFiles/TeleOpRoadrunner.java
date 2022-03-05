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

package testFiles;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "TeleOp Localization", group = "Iterative OpMode")

public class TeleOpRoadrunner extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive;

    int servoCount = 0;
    int helperCount = 0;
    Pose2d position;
    double posX;
    double posY;
    double posHead;
    TrajectorySequence autoTraj;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void loop() {
// Concept 1:
//        if(!gamepad1.a) {
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
//
////            Pose2d position = drive.getPoseEstimate();
////            double posX = position.getX();
////            double posY = position.getY();
////            double posHead = position.getHeading();
//
//
//            drive.update();
//        } else {
//            if (gamepad1.x) return;
////            drive.followTrajectorySequence(leftTrajectory);
//        }

        // Concept 2:
        switch (helperCount) {
            case 0:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                position = drive.getPoseEstimate();
                posX = position.getX();
                posY = position.getY();
                posHead = position.getHeading();

                drive.update();

                helperCount = (gamepad1.a ? 1 : 0);
                break;
            case 1:
                helperCount = (!gamepad1.a ? 2 : 1);
                break;
            case 2:
                position = drive.getPoseEstimate();

                autoTraj = drive.trajectorySequenceBuilder(position)
                        .strafeRight(20)
                        .build();

                while(!gamepad1.a) {
                    drive.followTrajectorySequence(autoTraj);
                }
//                } else {
//                    helperCount = 0;
//                    break;
//                }

                position = drive.getPoseEstimate();
                posX = position.getX();
                posY = position.getY();
                posHead = position.getHeading();

                drive.update();
                helperCount = (gamepad1.a ? 3 : 2);
                helperCount = 0;
                break;
            case 3:
                helperCount = (!gamepad1.a ? 0 : 3);
        }

        telemetry.addData("x", posX);
        telemetry.addData("y", posY);
        telemetry.addData("heading", posHead);
        telemetry.addData("helperCount = ", helperCount);
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
