package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by swamp on 10/17/2017.
 */

@Autonomous(name = "Blue Corner", group = "Autonomous")
public class TRAutoBlueCorner extends LinearOpMode {

    // Hardware class instance
    private TileRunnerREV hardware = new TileRunnerREV();

    // Vuforia instance
    private VuforiaLocalizer vuforia;

    // Inches required to knock a jewel off
    private final double JEWEL_INCHES = 4.0;

    // Jewel color enum
    JewelColor jewelColor;

    // IMU object
    BNO055IMU imu;

//    private double timeSnapshot = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("DO NOT PRESS PLAY! Hardware and Vuforia are being initialized.");
        telemetry.update();

        // Initialize hardware class
        hardware.init(hardwareMap);


        // Set up the Vuforia parameters
        // (Comment out first line and cameraMonitorViewId parameter on second line to remove screen display)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set the license key and the camera being used
        parameters.vuforiaLicenseKey = "AQEp+gX/////AAAAGabZ3yaKT0WLtofdjrrGznRKhqhzUCjAtaxsfr96aQv7kVlGcd6NUnv2Ic89/rJ/yPFvXrDDIWqGfpXvAhqVO94fs5EYBWUzB8qCBfTJ6U1Lmo15bBZ5/tz0iMkFc3ZX27xBTdIJ6C3zTIna1hErBvkeKpRI8nMwygPulWQej4jCaomF600Z9t9ZZZtQCH54bgqLmzMRIwZYOxCzcwh+nfP7teg9JtwI3NSUHmL2zkIiRYzwo53vv3+kv3CdzLnfiK/6ReAW6S/p9hO0ENCIcWJDUGgM4KDBW1aewp6OTpFt34D2ZIzop63/+ediGz8PJw3pcrRAuKEDQ/p1h7GAVHw8vbWgW1iTOkevHSv4bcp8\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Create a VuforiaLocalizer with the above parameters
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Get all the vision targets
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        // Get the template target
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        // Set the name (for debugging purposes)
        relicTemplate.setName("RelicVuMarkTemplate");


        // Set up the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // IMU parameters
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit             = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile   = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled        = true;
        IMUParameters.loggingTag            = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Pass in the parameters
        imu.initialize(IMUParameters);

        telemetry.addLine("Hardware Initialized.");
        telemetry.addLine("Press the play button to start.");
        telemetry.update();


        waitForStart();


        telemetry.addLine("Looking for vision target...");
        telemetry.update();
        // Start looking for the vision targets
        relicTrackables.activate();

        // Try to find the vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//        timeSnapshot = getRuntime();

        while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN/* && (getRuntime() - timeSnapshot) < 5*/) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

//        if(getRuntime() - timeSnapshot >= 5) {
//            telemetry.addLine("vuMark not found. Defaulting to center...");
//            telemetry.update();
//            sleep(1000);
//        }

        // Decide which color the jewel is

        hardware.jewelServo.setPosition(hardware.ARM_DOWN);
        sleep(1500);

        if (hardware.colorSensor.blue() > hardware.colorSensor.red()) {
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addLine("Left jewel was blue. Driving forwards...");
            telemetry.update();

            jewelColor = JewelColor.BLUE;

            sleep(1000);

            driveInches(0.4, JEWEL_INCHES);
        } else {
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addLine("Left jewel was red. Driving backwards...");
            telemetry.update();

            jewelColor = JewelColor.RED;

            sleep(1000);

            driveInches(0.4, -JEWEL_INCHES);
        }

        hardware.jewelServo.setPosition(hardware.ARM_UP);
        sleep(500);


        // Figure out how far to drive depending on
        // the cryptobox key and jewel knocked off
        double inches = 0.0;

        switch (vuMark) {
            case LEFT:
                inches = hardware.DIST_LEFT_CORNER      + (jewelColor == JewelColor.BLUE ? -4 : 4);
                break;
            case CENTER:
                inches = hardware.DIST_CENTER_CORNER    + (jewelColor == JewelColor.BLUE ? -4 : 4);
                break;
            case RIGHT:
                inches = hardware.DIST_RIGHT_CORNER     + (jewelColor == JewelColor.BLUE ? -4 : 4);
                break;
            default:
                telemetry.addLine("Vision target not found.");
                telemetry.update();
                sleep(1500);
        }

        telemetry.addLine(String.format("Driving to %s column...", vuMark));
        telemetry.update();
        driveInches(0.3, inches);

        telemetry.addLine("Turning towards cryptobox...");
        telemetry.update();
        sleep(1000);

        // Turn towards the cryptobox
        turnToHeadingPID(90);

        telemetry.addLine("Finished with turn.");
        telemetry.update();
        sleep(1000);


        telemetry.addLine("Spitting out the cube...");
        telemetry.update();
        sleep(1000);


        hardware.kicker.setPower(-0.8);
        sleep(250);
        hardware.kicker.setPower(0);


        hardware.setLifterPower(1.0);

        telemetry.addLine("Driving forward 6 inches...");
        telemetry.update();
        sleep(1000);

        driveInches(0.4, 6.0);

        telemetry.addLine("Driving backward 6 inches...");
        telemetry.update();
        sleep(1000);

        driveInches(0.4, -6.0);

        hardware.setLifterPower(0);


        telemetry.addLine("Finished.");
        telemetry.update();
        sleep(1000);


        while (opModeIsActive()) {
            telemetry.addLine("Visible vision target:");
            telemetry.addLine(vuMarkTelemetry(vuMark));
            telemetry.addLine();
            telemetry.addData("Jewel Color", jewelColor.toString());
            telemetry.addLine();
            telemetry.addLine(String.format("Distance to drive: %s inches", inches));
            telemetry.addLine();
            telemetry.addData("Gyro Heading", heading());
            telemetry.update();
        }
    }










    //----------------------------------------------------------------------------------------------
    // Driving and turning methods
    //----------------------------------------------------------------------------------------------
//    public void turnToHeading(double power, int heading) {
//        hardware.leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        telemetry.addLine(String.format("Turning to %s degrees...", heading));
//        telemetry.addLine(String.format("Start angle: %s", angles.firstAngle));
//        telemetry.update();
//        sleep(1500);
//
//        hardware.linearDrive(power, -power);
//
//        while(opModeIsActive() && angles.firstAngle < heading) {
//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            telemetry.addLine(String.format("Turning to %s degrees...", heading));
//            telemetry.addLine(String.format("Current angle: %s", angles.firstAngle));
//            telemetry.update();
//        }
//
//        hardware.linearDrive(0);
//
//        sleep(50);
//
//        hardware.leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hardware.leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hardware.rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hardware.rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        telemetry.addLine("Finished with turn.");
//        telemetry.update();
//        sleep(2000);
//    }


    public void turnToHeadingPID(int target) throws InterruptedException {
        hardware.pid.setSetpoint(target);                                       // Set target final heading relative to current
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);   // Set maximum motor power
        hardware.pid.setDeadband(hardware.TOLERANCE);                           // Set how far off you can safely be from your target

        while (opModeIsActive()) {
            double error = normalize180(target - heading());
            double power = hardware.pid.calculateGivenError(error);

            hardware.setLeftPower(power);
            hardware.setRightPower(-power);

            if (Math.abs(error) < hardware.TOLERANCE) {
                break;
            }

            Thread.sleep(1);
        }

        hardware.linearDrive(0);
    }

    public double normalize180(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double heading() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void driveEncoderCounts(double power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts);
//        telemetry.addData("Encoder counts", counts);
//        telemetry.addLine();
//        telemetry.addLine("Targets:");
//        telemetry.addLine();
//        telemetry.addData("Left drive 1", hardware.leftDrive1.getTargetPosition());
//        telemetry.addData("Left drive 2", hardware.leftDrive2.getTargetPosition());
//        telemetry.addData("Right drive 1", hardware.rightDrive1.getTargetPosition());
//        telemetry.addData("Right drive 2", hardware.rightDrive2.getTargetPosition());
//        telemetry.addLine();
//        telemetry.addLine("Current:");
//        telemetry.addLine();
//        telemetry.addData("Left drive 1", hardware.leftDrive1.getCurrentPosition());
//        telemetry.addData("Left drive 2", hardware.leftDrive2.getCurrentPosition());
//        telemetry.addData("Right drive 1", hardware.rightDrive1.getCurrentPosition());
//        telemetry.addData("Right drive 2", hardware.rightDrive2.getCurrentPosition());
//        telemetry.update();
//
//        sleep(1000);


        // Set run mode to RUN_TO_POSITION
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers to the specified amount
        hardware.linearDrive(power);

        // Run while op mode is active and motors are busy
        while (opModeIsActive() && hardware.driveMotorsBusy()) {
            telemetry.addLine(String.format("Left Drive 1: target = %1$s, current = %2$s",
                    hardware.leftDrive1.getTargetPosition(),
                    hardware.leftDrive1.getCurrentPosition()));
            telemetry.addLine(String.format("Left Drive 2: target = %1$s, current = %2$s",
                    hardware.leftDrive2.getTargetPosition(),
                    hardware.leftDrive2.getCurrentPosition()));
            telemetry.addLine(String.format("Right Drive 1: target = %1$s, current = %2$s",
                    hardware.rightDrive1.getTargetPosition(),
                    hardware.rightDrive1.getCurrentPosition()));
            telemetry.addLine(String.format("Right Drive 2: target = %1$s, current = %2$s",
                    hardware.rightDrive2.getTargetPosition(),
                    hardware.rightDrive2.getCurrentPosition()));
            telemetry.update();
        }

        // Stop motors
        hardware.linearDrive(0.0);

        // Set run mode to RUN_WITHOUT_ENCODER
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveInches(double power, double inches) {
        driveEncoderCounts(power, (int) (inches * hardware.COUNTS_PER_INCH));
    }

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