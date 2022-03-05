package testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


@Autonomous(name="Pure Roadrunner Auto", group="Auto")
public class roadrunnerTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading(new Pose2d(-18,-45, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(6.6, () -> {
                    // raise arm + servo up
                })
                .addTemporalMarker(7.6, () -> {
                    // stop arm motor power + extend servo arm
                })
                .addTemporalMarker(7.8, () -> {
                    // score
                })
                .waitSeconds(2)
                .strafeLeft(5)
                .lineToSplineHeading(new Pose2d(-60, -45, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-60, -50))
                .addTemporalMarker(17.3, () -> {
                    // turn carousel motor
                })
                .addTemporalMarker(19.3, () -> {
                    // stop carousel motor
                })
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(90))
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

