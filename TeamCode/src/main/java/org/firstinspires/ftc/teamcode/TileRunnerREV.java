package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kawaiiPlat on 9/30/2017.
 */

public class TileRunnerREV {

    // Encoder variables
    public final double COUNTS_PER_REV              = 280.0;    // For a NeveRest 40 (7 cpr with a 40:1 gear ratio)
    public final double DRIVE_GEAR_REDUCTION        = 1.0;      // No gear reduction (would be < 1.0 if geared up)
    public final double WHEEL_DIAMETER_INCHES       = 4.0;      // For figuring circumference
    public final double WHEEL_CIRCUMFERENCE_INCHES  = WHEEL_DIAMETER_INCHES * 3.1415;
    public final double COUNTS_PER_INCH             = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES;


    // Speed control variables
    public final double SLOW = 0.25;
    public final double NORMAL = 0.4;
    public final double FAST = 1.0;

    public double driverSpeedMod = NORMAL;
    public double utilitySpeedMod = NORMAL;


    // Hardware map
    HardwareMap hwMap = null;


    // Motor objects
    public DcMotor leftDrive1   = null;
    public DcMotor leftDrive2   = null;

    public DcMotor rightDrive1  = null;
    public DcMotor rightDrive2  = null;

    public DcMotor lifter1      = null;
    public DcMotor lifter2      = null;

    public DcMotor emptyMotor = null;

    // Servo objects
    public CRServo flipper  = null;
    public CRServo kicker   = null;

    // Sensor objects
    public ColorSensor colorSensor  = null;


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
        flipper = hwMap.crservo.get("flipper");
        kicker  = hwMap.crservo.get("kicker");


        // Get the sensors
        colorSensor = hwMap.colorSensor.get("color_sensor");


        // Set the motor directions
        leftDrive1.setDirection (DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection (DcMotorSimple.Direction.REVERSE);

        rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        lifter1.setDirection    (DcMotorSimple.Direction.FORWARD);
        lifter2.setDirection    (DcMotorSimple.Direction.REVERSE);


        // Set the motor powers to zero
        leftDrive1  .setPower(0);
        leftDrive2  .setPower(0);

        rightDrive1 .setPower(0);
        rightDrive2 .setPower(0);

        lifter1     .setPower(0);
        lifter2     .setPower(0);


        // Set the behavior of the motors when the power is set to zero
        leftDrive1.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive2.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);

        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lifter1.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        lifter2.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);


        // Set the motors' run mode
        leftDrive1.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive1.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter1.setMode     (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter2.setMode     (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /////////////////////////////////////
    // OpMode Methods
    /////////////////////////////////////

    public void linearDrive(float power) {
        linearDrive(power, power);
    }

    public void linearDrive(float leftPower, float rightPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setLeftPower(float leftPower) {
        leftDrive1.setPower(leftPower * driverSpeedMod);
        leftDrive2.setPower(leftPower * driverSpeedMod);
    }

    public void setRightPower(float rightPower) {
        rightDrive1.setPower(rightPower * driverSpeedMod);
        rightDrive2.setPower(rightPower * driverSpeedMod);
    }

    public void rampDrive(float leftPower, float rightPower) {
        setLeftPower((float) (leftPower * leftPower * leftPower * driverSpeedMod));
        setRightPower((float) (rightPower * rightPower * rightPower * driverSpeedMod));
    }

    public void resetDriveEncoders() {
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}