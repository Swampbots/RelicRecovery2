package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * https://github.com/Swampbots/2017offSeason.git
 * Created by kawaiiPlat on 7/29/2017.
 */

@TeleOp(name = "Beetle Drive", group = "TeleOp")
@Disabled
public class DemoBotTeleOp extends OpMode {

    // Hardware map initialization.
    DemoBot hardware = new DemoBot();


    // Runs once when the driver presses init.
    @Override
    public void init() {

        telemetry.addLine("Initializing hardware... do not press play!");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware initialized.");
        telemetry.update();
    }


    // Runs repeatedly after the driver presses init.
    @Override
    public void init_loop() {}


    // Runs once when the driver presses start.
    @Override
    public void start() {}


    // Runs repeatedly after the driver presses start.
    @Override
    public void loop() {

        // Handle speed modifiers
        if(gamepad1.left_bumper)        hardware.driverSpeedMod = hardware.FAST;
        else if(gamepad1.right_bumper)  hardware.driverSpeedMod = hardware.SLOW;
        else                            hardware.driverSpeedMod = hardware.NORMAL;

        if(gamepad2.left_bumper)        hardware.utilitySpeedMod = hardware.FAST;
        else if(gamepad2.right_bumper)  hardware.utilitySpeedMod = hardware.SLOW;
        else                            hardware.utilitySpeedMod = hardware.NORMAL;

        // Handle drive motors
        hardware.leftDrive1.setPower(gamepad1.left_stick_y * hardware.driverSpeedMod);
        hardware.rightDrive1.setPower(gamepad1.right_stick_y * hardware.utilitySpeedMod);

        // Update telemetry
        telemetry.addData("Runtime", getRuntime());
        telemetry.addLine();
        telemetry.addData("Driver Speed Mod",  hardware.driverSpeedMod);
        telemetry.addData("Utility Speed Mod",  hardware.utilitySpeedMod);
        telemetry.addLine();
        telemetry.addData("Red", hardware.colorSensor.red());
        telemetry.addData("Blue", hardware.colorSensor.blue());
        telemetry.addData("Green", hardware.colorSensor.green());
    }
}
