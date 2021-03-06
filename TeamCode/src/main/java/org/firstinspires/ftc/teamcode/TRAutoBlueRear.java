package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by SwampbotsAdmin on 10/31/2017.
 */

@Autonomous(name = "Blue Rear", group = "Autonomous")
public class TRAutoBlueRear extends LinearOpMode {

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

    // Time snapshot
    private double timeSnapshot = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("DO NOT PRESS PLAY! Hardware and Vuforia are being initialized.");
        telemetry.update();

        // Initialize hardware class
        hardware.init(hardwareMap);

        // Set the drive motors to brake
        hardware.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        BNO055IMU.Parameters IMUParameters  = new BNO055IMU.Parameters();
        IMUParameters.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit             = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile   = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled        = true;
        IMUParameters.loggingTag            = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Pass in the parameters
        imu.initialize(IMUParameters);


        while(!isStarted()) {
            telemetry.addLine("Hardware Initialized.");
            telemetry.addLine("Press the play button to start.");
            telemetry.addData("Heading", heading());
            telemetry.update();
        }

        waitForStart();




        telemetry.addLine("Looking for vision target...");
        telemetry.update();

        // Start looking for the vision targets
        relicTrackables.activate();

        // Take a snapshot of the time
        timeSnapshot = getRuntime();

        // Try to find the vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        // Keep looking for five seconds
        while (opModeIsActive()
                && vuMark == RelicRecoveryVuMark.UNKNOWN
                && (getRuntime() - timeSnapshot) < hardware.VUMARK_TIMEOUT) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            telemetry.addData("Search time", getRuntime() - timeSnapshot);
            telemetry.addData("Vision Target", hardware.vuMarkTelemetry(vuMark));
            telemetry.update();
        }

        // Default to center if the vision target is not found
        if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addLine("vuMark not found. Defaulting to center...");
            telemetry.update();
            sleep(1000);

            vuMark = RelicRecoveryVuMark.CENTER;
        }

        // Decide which color the jewel is

        hardware.jewelServo.setPosition(hardware.ARM_DOWN);
        sleep(hardware.ARM_TIME_SETTLE);

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

        driveInches(0.6, 25.0 + (jewelColor == JewelColor.BLUE ? -JEWEL_INCHES : (JEWEL_INCHES + 2)));

        telemetry.addLine("Turning parallel to cryptobox...");
        telemetry.update();
        turnToHeadingPID(-90);
        turnToHeadingPID(-90);

        double inches = 0.0;

        switch(vuMark) {
            case LEFT:
                inches = hardware.DIST_NEAR_REAR;
                break;
            case CENTER:
                inches = hardware.DIST_CENTER_REAR;
                break;
            case RIGHT:
                inches = hardware.DIST_FAR_REAR;
                break;
            default:
                telemetry.addLine("Vision target not found.");
                telemetry.update();
                inches = hardware.DIST_CENTER_REAR;
                sleep(2000);
        }

        driveInches(0.6, inches);

        telemetry.addLine(String.format("Aligned with %s column.", hardware.vuMarkTelemetry(vuMark)));
        telemetry.addLine("Turning towards cryptobox...");
        telemetry.update();
        turnToHeadingPID(0);
        turnToHeadingPID(0);

        telemetry.addLine("Spitting out the glyph...");
        telemetry.update();
        hardware.setLifterPower(hardware.SPEED_GLYPH_PLACE);
        sleep(1000);

//        driveInches(0.3, hardware.DIST_GLYPH_PLACE);
//
//        hardware.setLifterPower(1.0);
//
//        driveInches(0.3, hardware.DIST_GLYPH_RETURN);

        hardware.linearDrive(-0.3);
        sleep(1000);
        hardware.linearDrive(0);

        hardware.setLifterPower(0.8);
        sleep(1000);

        hardware.linearDrive(0.3);
        sleep(500);
        hardware.linearDrive(0);

        hardware.setLifterPower(0);

        while(opModeIsActive()) {
            telemetry.addLine("Vision target:");
            telemetry.addLine(hardware.vuMarkTelemetry(vuMark));
            telemetry.addLine();
            telemetry.addData("Drive distance", inches);
            telemetry.addLine();
            telemetry.addData("Jewel Color", jewelColor.toString());
            telemetry.addLine();
            telemetry.addData("Gyro Heading", heading());
            telemetry.update();
        }
    }









    //----------------------------------------------------------------------------------------------
    // Driving and turning methods
    //----------------------------------------------------------------------------------------------

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
}
