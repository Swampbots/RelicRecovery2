package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kawaiiPlat on 6/10/2017.
 */

@Autonomous(name = "Test Autonomous", group = "Test Autonomous")
public class TestAutonomous extends LinearOpMode {

    private TileRunnerREV hardware = new TileRunnerREV();

    private final float SPEED   = (float)0.7;
    //private final float INCHES  = (float)2.0;
    private final int COUNTS    = 979;


    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing hardware... do not press play!");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware initialized.");
        telemetry.update();


        waitForStart();


        while(opModeIsActive()) {

            // Handle telemetry
            telemetry.addLine("In end loop.");
//            telemetry.addData("Red", hardware.colorSensor.red());
//            telemetry.addData("Blue", hardware.colorSensor.blue());
//            telemetry.addData("leftIsBlue", hardware.colorSensor.blue() > hardware.colorSensor.red());
//            telemetry.addLine();
//            telemetry.addLine();
            telemetry.addData("Jewel Servo Position", hardware.jewelServo.getPosition());
            telemetry.addLine();
            telemetry.addData("Left Encoder 1", hardware.leftDrive1.getCurrentPosition());
            telemetry.addData("Left Encoder 2", hardware.leftDrive2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Right Encoder 1", hardware.rightDrive1.getCurrentPosition());
            telemetry.addData("Right Encoder 2", hardware.rightDrive2.getCurrentPosition());
            telemetry.update();

            // Perform testing commands
            if      (gamepad2.x) hardware.jewelServo.setPosition(hardware.jewelServo.getPosition() - 0.015);
            else if (gamepad2.y) hardware.jewelServo.setPosition(hardware.jewelServo.getPosition() + 0.015);
            else if (gamepad2.a) hardware.jewelServo.setPosition(0.5);

            if (gamepad2.dpad_up) {
                hardware.resetDriveEncoders();
            } else if (gamepad2.dpad_left) {
//                telemetry.addLine(String.format("Moving %1$s encoder counts (%2$s inches)...", (int) (INCHES * hardware.COUNTS_PER_INCH), INCHES));
//                telemetry.update();
//                driveInches(SPEED, INCHES);
                telemetry.addLine(String.format("Turning %s encoder counts...",-COUNTS));
                telemetry.update();
                turnEncoderCounts((float)1.0, -COUNTS);
                sleep(1000);
            } else if (gamepad2.dpad_right) {
//                telemetry.addLine(String.format("Moving %1$s encoder counts (%2$s inches)...", (int) (-INCHES * hardware.COUNTS_PER_INCH), -INCHES));
//                telemetry.update();
//                driveInches(SPEED, -INCHES);
                telemetry.addLine(String.format("Turning %s encoder counts...",COUNTS));
                telemetry.update();
                turnEncoderCounts((float)1.0, COUNTS);
                sleep(1000);
            }
        }
    }

    public void turnEncoderCounts(float power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts, true);

        // Set run mode to RUN_TO_POSITION
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers to the specified amount
        hardware.linearDrive(power);

        // Run while op mode is active and motors are busy
        while(opModeIsActive() && hardware.driveMotorsBusy());

        // Stop motors
        hardware.linearDrive((float)0.0);

        // Set run mode to RUN_WITHOUT_ENCODER
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void driveEncoderCounts(float power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts, false);

        // Set run mode to RUN_TO_POSITION
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers to the specified amount
        hardware.linearDrive(power);

        // Run while op mode is active and motors are busy
        while(opModeIsActive() && hardware.driveMotorsBusy());

        // Stop motors
        hardware.linearDrive((float)0.0);

        // Set run mode to RUN_WITHOUT_ENCODER
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveInches(float power, float inches) {
        driveEncoderCounts(power, (int)(inches * hardware.COUNTS_PER_INCH));
    }

}