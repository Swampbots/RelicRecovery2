package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by kawaiiPlat on 9/30/2017.
 */

public class TileRunnerREV {

    // Encoder variables
    private final double COUNTS_PER_REV             = 1120.0;   // For a NeveRest 40 (7 cpr with 4 pulses per count and a 40:1 gear ratio)
    private final double DRIVE_GEAR_REDUCTION       = 1.0;      // No gear reduction (would be < 1.0 if geared up)
    private final double WHEEL_DIAMETER_INCHES      = 4.0;      // For figuring circumference
    private final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * 3.1415; // 12.566 inches with four-inch wheels
    public final double COUNTS_PER_INCH             = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_INCHES; // About 89 counts per inch


    // Servo position variables
    public final double ARM_DOWN    = 0.75;
    public final double ARM_UP      = 0.10;

    public final double CATCHER_HOLDING     = 0.95;
    public final double CATCHER_RELEASED    = 0.4;

    public final double TIGHTENER_HOLDING   = 1.0;
    public final double TIGHTENER_RELEASED  = 0.3;


    // Autonomous distance variables from audience stones
    public final double DIST_NEAR_AUDIENCE      =  27.0;
    public final double DIST_CENTER_AUDIENCE    =  34.0;
    public final double DIST_FAR_AUDIENCE       =  42.0;

    // Autonomous distance variables from rear stones
    public final double DIST_NEAR_REAR      =  5.0;
    public final double DIST_CENTER_REAR    =  12.0;
    public final double DIST_FAR_REAR       =  22.0;

    // Autonomous distance variables for glyph deployment
    public final double DIST_GLYPH_PLACE   = 10.0;
    public final double DIST_GLYPH_RETURN  = -6.0;

    // Autonomous PID variables
    public final double MAX_SPEED = 0.4;
    public final double P = 0.045;
    public final double I = 0.01;
    public final double D = 0.045;
    public final double TOLERANCE = 2;

    public final SynchronousPID pid = new SynchronousPID(P, I, D);

    // Autonomous VuMark search timeout
    public final double VUMARK_TIMEOUT      = 5.0;

    // Autonomous lifter speeds for glyph placement
    public final double SPEED_GLYPH_PLACE   = 0.4;

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

    public DcMotor stonePusher  = null;

    public DcMotor winch        = null;


    // Servo objects
    public Servo flipper            = null;
    public Servo kicker             = null;

    public Servo jewelServo         = null;

    public Servo failedExperiment   = null;

    public Servo leftSweeper        = null;
    public Servo rightSweeper       = null;

    public Servo catcher            = null;
    public Servo tightener          = null;

    public Servo waver              = null;



    // Sensor objects
    public ColorSensor colorSensor  = null;
    public ColorSensor glyphSensor  = null;

//    public BNO055IMU imu = null;


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

        stonePusher = hwMap.dcMotor.get("stone_pusher");

        winch       = hwMap.dcMotor.get("winch");


        // Get the servos
        flipper             = hwMap.servo.get("flipper");
        kicker              = hwMap.servo.get("kicker");

        jewelServo          = hwMap.servo.get("jewel_servo");

        failedExperiment    = hwMap.servo.get("failed_experiment");

        leftSweeper         = hwMap.servo.get("left_sweeper");
        rightSweeper        = hwMap.servo.get("right_sweeper");

        catcher             = hwMap.servo.get("catcher");

        tightener           = hwMap.servo.get("tightener");

        waver               = hwMap.servo.get("waver");



        // Get the sensors
        colorSensor = hwMap.colorSensor.get("color_sensor");
        glyphSensor = hwMap.colorSensor.get("glyph_color");



        // Set the motor directions
        leftDrive1.setDirection (DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection (DcMotorSimple.Direction.REVERSE);

        rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        lifter1.setDirection    (DcMotorSimple.Direction.REVERSE);
        lifter2.setDirection    (DcMotorSimple.Direction.FORWARD);

        stonePusher.setDirection(DcMotorSimple.Direction.REVERSE);

        winch.setDirection      (DcMotorSimple.Direction.REVERSE);


        // Set the motor powers to zero
        leftDrive1  .setPower(0);
        leftDrive2  .setPower(0);

        rightDrive1 .setPower(0);
        rightDrive2 .setPower(0);

        lifter1     .setPower(0);
        lifter2     .setPower(0);

        stonePusher .setPower(0);

        winch       .setPower(0);


        // Set the behavior of the motors when the power is set to zero
        leftDrive1.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive2.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);

        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lifter1.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        lifter2.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);

        stonePusher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winch.setZeroPowerBehavior      (DcMotor.ZeroPowerBehavior.BRAKE);


        // Set the motors' run mode
        leftDrive1.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive1.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter1.setMode     (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter2.setMode     (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stonePusher.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        winch.setMode       (DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set the servo positions
        catcher.setPosition(CATCHER_HOLDING);
    }




    //----------------------------------------------------------------------------------------------
    // Motor Control Methods
    //----------------------------------------------------------------------------------------------

    public void linearDrive(double power) {
        linearDrive(power, power);
    }

    public void linearDrive(double leftPower, double rightPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setLeftPower(double leftPower) {
        leftDrive1.setPower(leftPower);
        leftDrive2.setPower(leftPower);
    }

    public void setRightPower(double rightPower) {
        rightDrive1.setPower(rightPower);
        rightDrive2.setPower(rightPower);
    }

    public void setLifterPower(double power) {
        lifter1.setPower(power);
        lifter2.setPower(power);
    }

//    public void rampDrive(float leftPower, float rightPower) {
//        setLeftPower((float) (leftPower * leftPower * leftPower * driverSpeedMod));
//        setRightPower((float) (rightPower * rightPower * rightPower * driverSpeedMod));
//    }

    //----------------------------------------------------------------------------------------------
    // Servo Control Methods
    //----------------------------------------------------------------------------------------------

    public void toggleServo(Servo servo) {
        servo.setPosition(Math.abs(servo.getPosition() - 1.0));
    }



    //----------------------------------------------------------------------------------------------
    // Drive Motor Attribute Modification Methods
    //----------------------------------------------------------------------------------------------

    public void setDriveRunMode(DcMotor.RunMode runMode) {
        leftDrive1.setMode(runMode);
        leftDrive2.setMode(runMode);
        rightDrive1.setMode(runMode);
        rightDrive2.setMode(runMode);
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftDrive1.setZeroPowerBehavior(behavior);
        leftDrive2.setZeroPowerBehavior(behavior);
        leftDrive1.setZeroPowerBehavior(behavior);
        rightDrive2.setZeroPowerBehavior(behavior);
    }



    //----------------------------------------------------------------------------------------------
    // Encoder-Specific Methods
    //----------------------------------------------------------------------------------------------

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

    public void setDriveTargetPosition(int counts) {
        leftDrive1.setTargetPosition(leftDrive1.getCurrentPosition() - counts);
        leftDrive2.setTargetPosition(leftDrive2.getCurrentPosition() - counts);
        rightDrive1.setTargetPosition(rightDrive1.getCurrentPosition() - counts);
        rightDrive2.setTargetPosition(rightDrive2.getCurrentPosition() - counts);
    }

    public boolean driveMotorsBusy() {
        return  leftDrive1.isBusy() &&
                    leftDrive2.isBusy() &&
                    rightDrive1.isBusy() &&
                    rightDrive2.isBusy();
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous-Specific Methods
    //----------------------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------------------
    // Telemetry Methods
    //----------------------------------------------------------------------------------------------

    public String vuMarkTelemetry(RelicRecoveryVuMark mark) {
        switch (mark) {
            case LEFT:
                return "Left";
            case CENTER:
                return "Center";
            case RIGHT:
                return "Right";
            default:
                return "None";
        }
    }
}