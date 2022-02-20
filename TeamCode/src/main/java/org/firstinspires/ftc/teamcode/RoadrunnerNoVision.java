package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Test Program Auto", group="Auto")
public class RoadrunnerNoVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .splineToLinearHeading(new Pose2d(-20 ,-10, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(1000)
                .lineToLinearHeading(new Pose2d(0 ,-50, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(0 ,0, 0))
//                .forward(5)
                .build();

        waitForStart();
        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(myTrajectory);
    }
}

