package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Roadrunner Auto Vision", group="Auto")
public class roadrunnerTest extends LinearOpMode {
    OpenCvWebcam webcam;
    SampleMecanumDrive drive;
    intakeSetup Intake = new intakeSetup();

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Intake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        vision detector = new vision(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence leftTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading(new Pose2d(-18,-45, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(6.6, () -> {
                    // raise arm to lowest + servo up
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

        TrajectorySequence middleTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading(new Pose2d(-18,-45, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(6.6, () -> {
                    // raise arm to middle level + servo up
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

        TrajectorySequence rightTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading(new Pose2d(-18,-45, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(6.6, () -> {
                    // raise arm to highest + servo up
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

        switch (detector.getLocation()) {
            case RIGHT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(rightTrajectory);
                break;
            case LEFT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(leftTrajectory);
                break;
            case CENTER:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(middleTrajectory);
                break;
        }
        webcam.stopStreaming();


        Intake.intakeMotor.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
//                .strafeRight(10)
//                .splineTo(new Vector2d(40, 40), Math.toRadians(90))
//                .forward(5)
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectory(myTrajectory);
    }
}

