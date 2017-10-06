package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kawaiiPlat on 9/30/2017.
 */

public class TileRunnerREV {

    // Speed control variables
    public final double SLOW = 0.25;
    public final double NORMAL = 0.4;
    public final double FAST = 1.0;

    public double driverSpeedMod = NORMAL;
    public double utilitySpeedMod = NORMAL;

    HardwareMap hwMap = null;

    // Motor objects
    public DcMotor leftDrive1   = null;
    public DcMotor leftDrive2   = null;

    public DcMotor rightDrive1  = null;
    public DcMotor rightDrive2  = null;

    public DcMotor lifter1      = null;
    public DcMotor lifter2      = null;


    // Servo objects
    public CRServo jewelServo = null;



    public TileRunnerREV() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Get the motors
        leftDrive1  = hwMap.dcMotor.get("left_drive_1");
        leftDrive2  = hwMap.dcMotor.get("left_drive_2");

        rightDrive1 = hwMap.dcMotor.get("right_drive_1");
        rightDrive2 = hwMap.dcMotor.get("right_drive_2");

        lifter1     = hwMap.dcMotor.get("lifter1");
        lifter2     = hwMap.dcMotor.get("lifter2");

        // Get the servos
        jewelServo  = hwMap.crservo.get("jewel_servo");

        // Set the motor directions
        leftDrive1.setDirection (DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection (DcMotorSimple.Direction.REVERSE);

        rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        lifter1.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter2.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set the motor powers to zero
        leftDrive1.setPower(0);
        leftDrive2.setPower(0);

        rightDrive1.setPower(0);
        rightDrive2.setPower(0);

        lifter1.setPower(0);
        lifter2.setPower(0);


        // Set the behavior of the motors when the power is set to zero
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set the motors' encoder usage
        leftDrive1.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive1.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter1.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter2.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /////////////////////////////////////
    // TeleOp methods
    /////////////////////////////////////

    public void linearDrive(float leftStickY, float rightStickY) {
        setLeftPower(leftStickY);
        setRightPower(rightStickY);
    }

    public void setLeftPower(float leftStickY) {
        leftDrive1.setPower(leftStickY * driverSpeedMod);
        leftDrive2.setPower(leftStickY * driverSpeedMod);
    }

    public void setRightPower(float rightStickY) {
        rightDrive1.setPower(rightStickY * driverSpeedMod);
        rightDrive2.setPower(rightStickY * driverSpeedMod);
    }


    public void rampDrive(float leftStickY, float rightStickY) {
        leftDrive1.setPower(leftStickY * leftStickY * leftStickY * driverSpeedMod);
        leftDrive2.setPower(leftStickY * leftStickY * leftStickY * driverSpeedMod);

        rightDrive1.setPower(rightStickY * rightStickY * rightStickY * driverSpeedMod);
        rightDrive2.setPower(rightStickY * rightStickY * rightStickY * driverSpeedMod);

    }
}