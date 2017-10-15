package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by captainFlareon 6/10/2017.
 */

@TeleOp(name = "Driver Control", group = "TeleOp")
public class TileRunnerTeleOp extends OpMode {
    // Hardware map initialization.
    private TileRunnerREV hardware = new TileRunnerREV();

    @Override
    public void init() {
        telemetry.addLine("Initializing hardware... do not press play!");
        telemetry.update();

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware initialized.");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}


    @Override
    public void loop() {
        // Handle speed modifiers
        if(gamepad1.left_bumper)        hardware.driverSpeedMod = hardware.FAST;
        else if(gamepad1.right_bumper)  hardware.driverSpeedMod = hardware.SLOW;
        else                            hardware.driverSpeedMod = hardware.NORMAL;

        if(gamepad2.left_bumper)        hardware.utilitySpeedMod = hardware.FAST;
        else if(gamepad2.right_bumper)  hardware.utilitySpeedMod = hardware.SLOW;
        else                            hardware.utilitySpeedMod = hardware.NORMAL;

        // Handle motors
        hardware.linearDrive(gamepad1.left_stick_y, gamepad2.right_stick_y);

        hardware.lifter1.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);
        hardware.lifter2.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);

        hardware.wheelIntake1.setPower(gamepad2.right_stick_y * hardware.utilitySpeedMod);
        hardware.wheelIntake2.setPower(gamepad2.right_stick_y * hardware.utilitySpeedMod);

        // Handle servos
        if(gamepad2.x) hardware.flipper.setPower(-1.0 * hardware.utilitySpeedMod);
        else if(gamepad2.y) hardware.flipper.setPower(1.0 * hardware.utilitySpeedMod);
        else hardware.flipper.setPower(0.0 * hardware.utilitySpeedMod);

        if(gamepad2.a) hardware.kicker.setPower(-1.0 * hardware.utilitySpeedMod);
        else if(gamepad2.b) hardware.kicker.setPower(1.0 * hardware.utilitySpeedMod);
        else hardware.kicker.setPower(0.0 * hardware.utilitySpeedMod);

        hardware.linearDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

        // Update telemetry
        telemetry.addData("Driver Speed Mod",  hardware.driverSpeedMod);
        telemetry.addData("Utility Speed Mod",  hardware.utilitySpeedMod);
        telemetry.update();
    }
}