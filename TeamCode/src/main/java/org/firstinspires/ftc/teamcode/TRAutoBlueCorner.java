package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by swamp on 10/17/2017.
 */

@Disabled
@Autonomous(name = "Blue Corner", group = "Blue Autonomous")
public class TRAutoBlueCorner extends LinearOpMode {

    private TileRunnerREV hardware = new TileRunnerREV();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing telemetry... do not press play.");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware Initialized.");
        telemetry.addLine("Press the play button to start.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addLine("Color Sensor Values:");
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.update();
        }
    }

    public void driveEncoderCounts(double power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts);

        // Set run mode to RUN_TO_POSITION
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers to the specified amount
        hardware.linearDrive((float)power);

        // Run while op mode is active and motors are busy
        while(opModeIsActive() && hardware.driveMotorsBusy()) {
            telemetry.addLine(String.format("Moving %s encoder counts...", counts));
            telemetry.update();
        }

        // Stop motors
        hardware.linearDrive((float)0.0);

        // Set run mode to RUN_WITHOUT_ENCODERS
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveInches(float power, float inches) {
        driveEncoderCounts(power, (int)(inches * hardware.COUNTS_PER_INCH));
    }
}
