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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class liftSetup
{
    // Motor Constructors
    public DcMotorEx liftMotor = null;


    HardwareMap hwMap =  null;

    public liftSetup() {

    }

    // Setup function used to hardwareMap motor
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        // Hardware Map
        liftMotor = hwMap.get(DcMotorEx.class, "liftM");
        // Set direction
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
//    static double speed = 1200;
//
//    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0,0,0);
//    public PIDCoefficients pidGains = new PIDCoefficients(0,0,0);
//
//    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//    @Override
//    public void runOpMode() {
//        liftMotor = hardwareMap.get(DcMotorEx.class, "liftM");
//
//        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
//
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//
//        if(opModeIsActive()) {
//            while(opModeIsActive()) {
//                PID(speed);
//                telemetry.addData("RPM: ", liftMotor.getVelocity());
//                telemetry.update();
//            }
//        }
//    }
//
//    double integral = 0;
//    double lastError = 0;
//
//    public void PID(double targetVelocity) {
//
//        PIDTimer.reset();
//
//        double currentVelocity = liftMotor.getVelocity();
//
//        double error = targetVelocity - currentVelocity;
//
//        integral += error * PIDTimer.time();
//
//        double deltaError = error - lastError;
//        double derivative = deltaError / PIDTimer.time();
//
//        pidGains.p = pidCoeffs.p * error;
//        pidGains.i = pidCoeffs.i * integral;
//        pidGains.d = pidGains.d * derivative;
//
//        liftMotor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
//
//        lastError = error;
//    }
}

