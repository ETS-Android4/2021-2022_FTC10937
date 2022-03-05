package testFiles;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardwareSetup.driveTrainSetup;
import org.firstinspires.ftc.teamcode.hardwareSetup.intakeSetup;
import org.firstinspires.ftc.teamcode.hardwareSetup.vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class automaticDetectionAuto extends LinearOpMode {
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
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        runtime.reset();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        switch (detector.getLocation()) {
            case LEFT:
                if (isStopRequested()) return;

                break;
            case RIGHT:
                if (isStopRequested()) return;

                break;
            case CENTER:
                if (isStopRequested()) return;

                break;
        }
    }
}
