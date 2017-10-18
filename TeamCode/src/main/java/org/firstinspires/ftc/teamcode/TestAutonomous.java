package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kawaiiPlat on 6/10/2017.
 */

@Autonomous(name = "Test Autonomous", group = "Test Autonomous")
public class TestAutonomous extends LinearOpMode {

    private TileRunnerREV hardware = new TileRunnerREV();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing hardware... do not press play!");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware initialized.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addLine("In end loop.");
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("leftIsBlue", hardware.colorSensor.blue() > hardware.colorSensor.red());
            telemetry.update();
        }
    }
}