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

    public final double SERVO_TIMEOUT   = 0.500; // 500 milliseconds

    public boolean waveServo = false;

    double lastChangeFlipper = 0.0;
    double lastChangeKicker = 0.0;
    double lastChangeLeftSweeper = 0.0;
    double lastChangeRightSweeper = 0.0;


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
        //------------------------------------------------------------------------------------------
        // Speed modifiers
        //------------------------------------------------------------------------------------------

        if(gamepad1.left_bumper)        hardware.driverSpeedMod = hardware.FAST;
        else if(gamepad1.right_bumper)  hardware.driverSpeedMod = hardware.SLOW;
        else                            hardware.driverSpeedMod = hardware.NORMAL;

        if(gamepad2.left_bumper)        hardware.utilitySpeedMod = hardware.FAST;
        else if(gamepad2.right_bumper)  hardware.utilitySpeedMod = hardware.SLOW;
        else                            hardware.utilitySpeedMod = hardware.NORMAL;



        //------------------------------------------------------------------------------------------
        // Motor Controls
        //------------------------------------------------------------------------------------------

        hardware.linearDrive(
                (gamepad1.left_stick_y * hardware.driverSpeedMod),
                (gamepad1.right_stick_y * hardware.driverSpeedMod));

        hardware.lifter1.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);
        hardware.lifter2.setPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);

        hardware.winch.setPower(gamepad2.right_stick_y * hardware.utilitySpeedMod);



        //------------------------------------------------------------------------------------------
        // Servo controls
        //------------------------------------------------------------------------------------------


        if(gamepad2.x && getRuntime() - lastChangeFlipper > SERVO_TIMEOUT) {
            lastChangeFlipper = getRuntime();
            hardware.toggleServo(hardware.flipper);
        }

        if(gamepad2.b && getRuntime() - lastChangeKicker > SERVO_TIMEOUT) {
            lastChangeKicker = getRuntime();
            hardware.toggleServo(hardware.kicker);
        }



        if      (gamepad2.dpad_right)   hardware.jewelServo.setPosition(hardware.ARM_UP);
        else if (gamepad2.dpad_left)    hardware.jewelServo.setPosition(hardware.ARM_DOWN);

        if      (gamepad2.dpad_up)      hardware.failedExperiment.setPosition(0.0);
        else if (gamepad2.dpad_down)    hardware.failedExperiment.setPosition(1.0);

        if      (gamepad2.right_stick_button)   hardware.catcher.setPosition(hardware.CATCHER_HOLDING);
        if      (gamepad2.left_stick_button)    hardware.catcher.setPosition(hardware.CATCHER_RELEASED);

        if      (gamepad1.x && getRuntime() - lastChangeLeftSweeper > SERVO_TIMEOUT) {
            lastChangeLeftSweeper = getRuntime();
            hardware.toggleServo(hardware.leftSweeper);
        }

        if      (gamepad1.b && getRuntime() - lastChangeRightSweeper > SERVO_TIMEOUT) {
            lastChangeRightSweeper = getRuntime();
            hardware.toggleServo(hardware.rightSweeper);
        }

        if(gamepad1.a)  hardware.tightener.setPosition(hardware.TIGHTENER_HOLDING);
        if(gamepad1.y)  hardware.tightener.setPosition(hardware.TIGHTENER_RELEASED);

        if(gamepad1.left_stick_button) waveServo = !waveServo;

        if(waveServo) {
            if(hardware.waver.getPosition() == 0.4) hardware.waver.setPosition(hardware.waver.getPosition() + 0.01);
            if(hardware.waver.getPosition() == 0.6) hardware.waver.setPosition(hardware.waver.getPosition() - 0.01);
        }

        hardware.stonePusher.setPosition(gamepad1.right_trigger);


        // Update telemetry
        telemetry.addData("Driver Speed Mod",  hardware.driverSpeedMod);
        telemetry.addData("Utility Speed Mod",  hardware.utilitySpeedMod);
        telemetry.addLine();
        telemetry.addData("Tail Encoder", hardware.stonePusher.getPosition());
        telemetry.addData("Glyph Red", hardware.glyphSensor.red());
        telemetry.addData("Glyph Blue", hardware.glyphSensor.blue());
        telemetry.update();
    }
}