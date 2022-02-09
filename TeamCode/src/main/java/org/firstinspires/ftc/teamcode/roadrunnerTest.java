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

@Autonomous(name="Roadrunner Test Auto", group="Auto")
public class roadrunnerTest extends LinearOpMode {
    OpenCvWebcam webcam;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    intakeSetup Intake = new intakeSetup(hardwareMap);

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
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

        TrajectorySequence leftTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-44, -61, Math.toRadians(90)))
                .strafeRight(33)
                .forward(19)
                .addTemporalMarker(2.6, () -> {
                    //score element

                })
                .waitSeconds(1)
                .addTemporalMarker(3.6, () -> {
                    //intake stop

                })
                .waitSeconds(0.5)
                .addTemporalMarker(4.1, () -> {
                    //intake retract

                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .addTemporalMarker(6.5, () -> {
                    //spin carousel

                })
                .waitSeconds(1)
                .addTemporalMarker(7.5, () -> {
                    //carousel stop

                })
                .lineTo(new Vector2d(-60, -35))
                .build();

        TrajectorySequence middleTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                .strafeRight(24)
                .forward(19)
                .addTemporalMarker(2.4, () -> {
                    //score element

                })
                .waitSeconds(1)
                .addTemporalMarker(3.4, () -> {
                    //intake stop

                })
                .waitSeconds(0.5)
                .addTemporalMarker(3.4, () -> {
                    //intake retract

                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .addTemporalMarker(6.3, () -> {
                    //spin carousel

                })
                .waitSeconds(1)
                .addTemporalMarker(7.3, () -> {
                    //carousel stop

                })
                .lineTo(new Vector2d(-60, -35))
                .build();

        TrajectorySequence rightTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-27, -61, Math.toRadians(90)))
                .strafeRight(16)
                .forward(19)
                .addTemporalMarker(2.6, () -> {
                    //score element

                })
                .waitSeconds(1)
                .addTemporalMarker(3.6, () -> {
                    //intake stop

                })
                .waitSeconds(0.5)
                .addTemporalMarker(4.1, () -> {
                    //intake retract

                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .addTemporalMarker(6.1, () -> {
                    //spin carousel

                })
                .waitSeconds(1)
                .addTemporalMarker(7.1, () -> {
                    //carousel stop

                })
                .lineTo(new Vector2d(-60, -35))
                .build();

        waitForStart();
        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        switch (detector.getLocation()) {
            case LEFT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(leftTrajectory);
                break;
            case RIGHT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(rightTrajectory);
                break;
            case NOT_FOUND:
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

