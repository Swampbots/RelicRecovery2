package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by SwampbotsAdmin on 11/4/2017.
 */

@Disabled
@TeleOp(name = "Crazy Cat Ladies Driver Control", group = "TeleOp")
public class CrazyCatLadiesTeleOp extends OpMode {

    DcMotor frontLeft = null;

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("fl");
    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {
        if(gamepad1.a) frontLeft.setPower(1.0);
        else if(gamepad1.b) frontLeft.setPower(-1.0);
    }

}
