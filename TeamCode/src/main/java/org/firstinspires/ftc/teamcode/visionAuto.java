package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
@Autonomous(name="Color Detection Auto", group="Auto")
public class visionAuto extends LinearOpMode{
    OpenCvWebcam webcam;

    ElapsedTime runtime = new ElapsedTime();
    driveTrainSetup Drive = new driveTrainSetup(hardwareMap);
    intakeSetup Intake = new intakeSetup(hardwareMap);

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

        waitForStart();
        runtime.reset();

        Drive.left1.setPower(1);
        Drive.left2.setPower(1);
        Drive.right1.setPower(1);
        Drive.right2.setPower(1);
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        switch (detector.getLocation()) {
            case LEFT:
                Drive.left1.setPower(-1);
                Drive.left2.setPower(1);
                Drive.right1.setPower(1);
                Drive.right2.setPower(-1);
                break;
            case RIGHT:
                Drive.left1.setPower(1);
                Drive.left2.setPower(-1);
                Drive.right1.setPower(-1);
                Drive.right2.setPower(1);
                break;
            case NOT_FOUND:
                Drive.left1.setPower(0);
                Drive.left2.setPower(0);
                Drive.right1.setPower(0);
                Drive.right2.setPower(0);
                break;
        }
        webcam.stopStreaming();

        Drive.left1.setPower(1);
        Drive.left2.setPower(1);
        Drive.right1.setPower(1);
        Drive.right2.setPower(1);
        Intake.intakeMotor.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}
