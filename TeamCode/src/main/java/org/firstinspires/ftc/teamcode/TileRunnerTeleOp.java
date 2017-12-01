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

    public final double TIMEOUT = 0.500; // 500 milliseconds

    double lastChange = 0.0;

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
    public void start() {
        hardware.jewelServo.setPosition(hardware.ARM_UP);
    }


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
        hardware.linearDrive(
                (gamepad1.left_stick_y * hardware.driverSpeedMod),
                (gamepad1.right_stick_y * hardware.driverSpeedMod));

        hardware.lifter1.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);
        hardware.lifter2.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);

        hardware.stonePusher.setPower(hardware.driverSpeedMod * gamepad1.right_trigger);
        hardware.stonePusher.setPower(hardware.driverSpeedMod * -gamepad1.left_trigger);

        hardware.winch.setPower(gamepad2.right_stick_y * hardware.utilitySpeedMod);


        // Handle servos
        if      (gamepad2.a)    hardware.kicker.setPower(0.8);
        else if (gamepad2.b)    hardware.kicker.setPower(-0.8);
        else                    hardware.kicker.setPower(0.0);

        if      (gamepad2.x)    hardware.flipper.setPower(0.8);
        else if (gamepad2.y)    hardware.flipper.setPower(-0.8);
        else                    hardware.flipper.setPower(0.0);

        if      (gamepad2.dpad_right)   hardware.jewelServo.setPosition(hardware.ARM_UP);
        else if (gamepad2.dpad_left)    hardware.jewelServo.setPosition(hardware.ARM_DOWN);

        if      (gamepad2.dpad_up)      hardware.failedExperiment.setPosition(0.0);
        else if (gamepad2.dpad_down)    hardware.failedExperiment.setPosition(1.0);

        if      (gamepad2.right_stick_button)   hardware.catcher.setPosition(hardware.CATCHER_HOLDING);
        if      (gamepad2.left_stick_button)    hardware.catcher.setPosition(hardware.CATCHER_RELEASED);

        if      (gamepad1.x && getRuntime() - lastChange > TIMEOUT) {
            lastChange = getRuntime();
            hardware.leftSweeper.setPosition(Math.abs(hardware.leftSweeper.getPosition() - 1.0));
        }

        if      (gamepad1.b && getRuntime() - lastChange > TIMEOUT) {
            lastChange = getRuntime();
            hardware.rightSweeper.setPosition(Math.abs(hardware.rightSweeper.getPosition() - 1.0));
        }

        // Update telemetry
        telemetry.addData("Driver Speed Mod",  hardware.driverSpeedMod);
        telemetry.addData("Utility Speed Mod",  hardware.utilitySpeedMod);
        telemetry.addLine();
        telemetry.update();
    }
}