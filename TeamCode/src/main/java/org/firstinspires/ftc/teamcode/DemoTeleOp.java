package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by SwampbotsAdmin on 5/12/2018.
 */

@TeleOp(name = "Demo Driver Control", group = "TeleOp")
public class DemoTeleOp extends OpMode {
    private TileRunnerREV hardware = new TileRunnerREV();

    public final double SERVO_TIMEOUT = 0.500; // 500 milliseconds
    public final double DEMO_SPEED    = 0.8;

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

        hardware.jewelServo.setPosition(hardware.ARM_UP);

        telemetry.addLine("Hardware initialized.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {
        //------------------------------------------------------------------------------------------
        // Speed modifiers and State Variables
        //------------------------------------------------------------------------------------------

        if (gamepad1.left_bumper) hardware.driverSpeedMod = hardware.FAST * DEMO_SPEED;
        else if (gamepad1.right_bumper) hardware.driverSpeedMod = hardware.SLOW;
        else hardware.driverSpeedMod = hardware.NORMAL;

        if (gamepad2.left_bumper) hardware.utilitySpeedMod = hardware.FAST * DEMO_SPEED;
        else if (gamepad2.right_bumper) hardware.utilitySpeedMod = hardware.SLOW;
        else hardware.utilitySpeedMod = hardware.NORMAL;


        //------------------------------------------------------------------------------------------
        // Motor Controls
        //------------------------------------------------------------------------------------------

        // Drive Wheels
        hardware.linearDrive(
                (gamepad1.left_stick_y * hardware.driverSpeedMod),
                (gamepad1.right_stick_y * hardware.driverSpeedMod));

        // Lifter
        hardware.setLifterPower(gamepad2.left_stick_y * hardware.utilitySpeedMod);

//        // Winch
//        hardware.winch.setPower(gamepad2.right_stick_y * hardware.utilitySpeedMod);
//
//        // Pivot
//        if      (gamepad2.left_trigger < 0.6)   hardware.pivot.setPower(gamepad2.right_trigger);
//        else if (gamepad2.right_trigger < 0.6)  hardware.pivot.setPower(-gamepad2.left_trigger);


        //------------------------------------------------------------------------------------------
        // Servo controls
        //------------------------------------------------------------------------------------------


        // Flipper
        if (gamepad2.x && getRuntime() - lastChangeFlipper > SERVO_TIMEOUT) {
            lastChangeFlipper = getRuntime();
            hardware.toggleServo(hardware.flipper);
        }

        // Kicker
        if (gamepad2.b && getRuntime() - lastChangeKicker > SERVO_TIMEOUT) {
            lastChangeKicker = getRuntime();
            hardware.toggleServo(hardware.kicker);
        }


//        // Jewel Servo
//        hardware.jewelServo.setPosition(
//                gamepad2.dpad_left ? hardware.ARM_DOWN : hardware.ARM_UP
//        );


//        // Gripper
//        hardware.gripper.setPosition(
//                (gamepad2.dpad_down || gamepad1.y) ? hardware.GRIPPER_RELEASED : hardware.GRIPPER_ENGAGED
//        );


        // Left Sweeper
        if (gamepad1.x && getRuntime() - lastChangeLeftSweeper > SERVO_TIMEOUT) {
            lastChangeLeftSweeper = getRuntime();
            hardware.toggleServo(hardware.leftSweeper);
        }

        // Right Sweeper
        if      (gamepad1.b && getRuntime() - lastChangeRightSweeper > SERVO_TIMEOUT) {
            lastChangeRightSweeper = getRuntime();
            hardware.toggleServo(hardware.rightSweeper);
        }


//        // Waver
//        if(gamepad1.left_stick_button) waveServo = !waveServo;

//        if(waveServo) {
//            if(hardware.waver.getPosition() == 0.4) hardware.waver.setPosition(hardware.waver.getPosition() + 0.01);
//            if(hardware.waver.getPosition() == 0.6) hardware.waver.setPosition(hardware.waver.getPosition() - 0.01);
//        }


//        // Stone Pusher
//        hardware.stonePusher.setPosition(gamepad1.right_trigger);


        //------------------------------------------------------------------------------------------
        // Telemetry
        //------------------------------------------------------------------------------------------
        telemetry.addData("Driver Speed Mod", hardware.driverSpeedMod);
        telemetry.addData("Utility Speed Mod", hardware.utilitySpeedMod);
        telemetry.addLine();
        telemetry.addData("Tail Encoder", hardware.stonePusher.getPosition());
        telemetry.addData("Glyph Red", hardware.glyphSensor.red());
        telemetry.addData("Glyph Blue", hardware.glyphSensor.blue());
        telemetry.addLine();
        telemetry.addData("Jewel Red", hardware.colorSensor.red());
        telemetry.addData("Jewel Blue", hardware.colorSensor.blue());
        telemetry.update();
        telemetry.update();
    }
}
