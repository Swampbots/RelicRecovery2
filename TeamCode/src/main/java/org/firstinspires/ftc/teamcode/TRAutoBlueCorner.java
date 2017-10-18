package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
}
