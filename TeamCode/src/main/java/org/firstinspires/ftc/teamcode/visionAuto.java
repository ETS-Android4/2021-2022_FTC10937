package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="Color Detection Auto", group="Auto")
public class visionAuto extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;

        ElapsedTime runtime = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        driveTrainSetup drive1 = new driveTrainSetup();
        intakeSetup Intake = new intakeSetup();

        Intake.init(hardwareMap);
        drive1.init(hardwareMap);
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
                .strafeRight(10)
                .build();

        TrajectorySequence midTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-44, -61, Math.toRadians(90)))
                .back(5)
                .turn(180)
                .build();

        TrajectorySequence rightTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-44, -61, Math.toRadians(90)))
                .strafeLeft(10)
                .build();

        waitForStart();
        runtime.reset();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        switch (detector.getLocation()) {
            case LEFT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(leftTrajectory);
                break;
            case RIGHT:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(rightTrajectory);
                break;
            case CENTER:
                if (isStopRequested()) return;
                drive.followTrajectorySequence(midTrajectory);
                break;
        }
    }
}
