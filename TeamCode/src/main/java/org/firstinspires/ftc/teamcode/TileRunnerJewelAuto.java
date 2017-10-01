package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by kawaiiPlat on 9/14/2017.
 */

@Autonomous(name = "Jewel Test", group = "Autonomous")
public class TileRunnerJewelAuto extends LinearOpMode {
    TileRunnerMR hardware  = new TileRunnerMR();

    public void runOpMode() {
        telemetry.addLine("Initializing hardware... do not press start!");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware intitialized.");
        telemetry.addLine("Press the play button to start.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addLine("Finished.");
            telemetry.addLine("In loop.");
            telemetry.update();
        }


    }
}
