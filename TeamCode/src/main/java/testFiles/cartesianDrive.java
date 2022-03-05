package testFiles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Cartesian Test")
public class cartesianDrive extends OpMode {
    private boolean arcadeMode = false;
    private int gyroCalibratedCount = 0;
    gyroAndMotorSetup IMU = new gyroAndMotorSetup();

    @Override
    public void init() {
        IMU.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            arcadeMode = ! arcadeMode;
        }
        telemetry.addData("Gyro Ready?", IMU.isGyroCalibrated() ? "YES" : "no.");
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.update();
    }

    @Override
    public void loop() {
        IMU.loop();

        // resets gyro values
        if (gamepad1.x) {
            IMU.resetHeading();
        }
        // switches from different drive methods
        if (gamepad1.a) {
            arcadeMode = !arcadeMode;
        }
        // telemetry for robot status
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", IMU.getHeadingDegrees());
        telemetry.update();

        // get coordinate values from joysticks
        final double x = -(Math.pow(gamepad1.left_stick_x, 3.0));
        final double y = (Math.pow(gamepad1.left_stick_y, 3.0));

        // rotation variable from right stick x
        final double rotation = Math.pow(gamepad1.right_stick_x, 3.0);
        // use atan to calculate angle from x and y values
        final double direction = Math.atan2(x, y) + (arcadeMode ? IMU.getHeading() : 0.0);
        // calculate speed of robot
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        // calculate values for amount of speed for each motor
        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        // set motor powers to calculated speeds
        IMU.setMotors(lf, lr, rf, rr);
    }
}