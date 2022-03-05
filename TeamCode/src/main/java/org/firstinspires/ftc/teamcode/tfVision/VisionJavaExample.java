//package org.firstinspires.ftc.teamcode.tfVision;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//
//@TeleOp
//@Disabled
//public class VisionJavaExample extends LinearOpMode{
//    MasterVision vision;
//    SampleRandomizedPositions goldPosition;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
//        parameters.vuforiaLicenseKey = "AfkqKuT/////AAABmScS2VI1i0eXpv1kyo+BFENYUTTTgBUA89vumnZzOQmybKEWpF/sabHNkIsdReNr5+yZBSGmFH1reGh4DIcQtEhUcLL2T0pmFkU7xSZcfNTd9/HlffGWezZJ+DRsCEize+e3wTiPtwXrLA8KZO+2fBnWYUr6oLWhvEOa03PK+sfY7fl+2hpDgX5LBs9JzEWsZlEq5F58oDCBuNM7zQqs89CrOuGQIuYHcahTAfjl0kr+mNIIBO+YVJJOFNTscV2HWhqqYmi2qBeqt6Cad22RGS9pqtHPbfbQuEX8gk6oLd9PWYtgAPR3MSCbbF+vM9MlvGOrtREUd9Ya/Za3rHiUJOPkcakK9TGMThUvb1gP93X9";
//
//        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
//        vision.init();// enables the camera overlay. this will take a couple of seconds
//        vision.enable();// enables the tracking algorithms. this might also take a little time
//
//        waitForStart();
//
//        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.
//
//        goldPosition = vision.getTfLite().getLastKnownSampleOrder();
//
//        while(opModeIsActive()){
//            telemetry.addData("goldPosition was", goldPosition);// giving feedback
//
//            switch (goldPosition){ // using for things in the autonomous program
//                case LEFT:
//                    telemetry.addLine("going to the left");
//                    break;
//                case CENTER:
//                    telemetry.addLine("going straight");
//                    break;
//                case RIGHT:
//                    telemetry.addLine("going to the right");
//                    break;
//                case UNKNOWN:
//                    telemetry.addLine("staying put");
//                    break;
//            }
//
//            telemetry.update();
//        }
//
//        vision.shutdown();
//    }
//}
