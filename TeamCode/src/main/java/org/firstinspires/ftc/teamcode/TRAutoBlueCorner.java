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

@Autonomous(name = "Blue Corner", group = "Blue Autonomous")
public class TRAutoBlueCorner extends LinearOpMode {

    // Hardware class instance
    private TileRunnerREV hardware = new TileRunnerREV();

    // Vuforia instance
    private VuforiaLocalizer vuforia;

    // Jewel color enum
    JewelColor jewelColor;

    // IMU object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() {

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

        // Create a VuforiaLocalizer
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
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Pass in the parameters
        imu.initialize(IMUParameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



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

        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }


        // Decide which color the jewel is

        hardware.jewelServo.setPosition(hardware.ARM_DOWN);
        telemetry.addData("Blue", hardware.colorSensor.blue());
        telemetry.addData("Red", hardware.colorSensor.red());
        telemetry.update();
        sleep(1000);

        if(hardware.colorSensor.blue() > hardware.colorSensor.red()) {
            jewelColor = JewelColor.BLUE;
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addLine("Left jewel was blue. Driving forwards...");
            telemetry.update();
            sleep(1500);

            driveInches((float)0.3, (float)3.0);
        }
        else {
            jewelColor = JewelColor.RED;
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addLine("Left jewel was red. Driving backwards...");
            telemetry.update();
            sleep(1500);

            driveInches((float)0.3, (float)-3.0);
        }

        hardware.jewelServo.setPosition(hardware.ARM_UP);
        sleep(500);


        // Figure out how far to drive depending on
        // the cryptobox key and jewel knocked off
        float inches = (float)0.0;

        switch(vuMark) {
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
        driveInches((float)0.3, inches);
        sleep(1000);

        // Turn towards the cryptobox (with a 17 degree undershoot)
        if(angles != null) {
            turnToHeading((float)0.4, 73);
        } else {
            telemetry.addLine("Angles is null");
            telemetry.update();
            sleep(2000);
        }


        while(opModeIsActive()) {
            telemetry.addLine("Visible vision target:");
            telemetry.addLine(vuMarkTelemetry(vuMark));
            telemetry.addLine();
            telemetry.addData("Jewel Color", jewelColor.toString());
            telemetry.addLine();
            telemetry.addLine(String.format("Distance to drive: %s inches", inches));
            telemetry.addData("Gyro Heading", angles.firstAngle);

            telemetry.update();
        }
    }

    public void turnToHeading(float power, int heading) {
        hardware.leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine(String.format("Turning to %s degrees...", heading));
        telemetry.addLine(String.format("Start angle: %s", angles.firstAngle));
        telemetry.update();
        sleep(1500);

        hardware.linearDrive(power, -power);

        while(opModeIsActive() && angles.firstAngle < heading) {
            telemetry.addLine(String.format("Turning to %s degrees...", heading));
            telemetry.addLine(String.format("Current angle: %s", angles.firstAngle));
            telemetry.update();
        }
        hardware.linearDrive(0);

        telemetry.addLine("Finished with turn.");
        telemetry.update();
        sleep(2000);
    }

//    public void turnEncoderCounts(float power, int counts) {
//        // Set target positions
//        hardware.setDriveTargetPosition(counts, true);
//
//        // Set run mode to RUN_TO_POSITION
//        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set motor powers to the specified amount
//        hardware.linearDrive(power);
//
//        // Run while op mode is active and motors are busy
//        while(opModeIsActive() && hardware.driveMotorsBusy()) {
//            telemetry.addData("Left motor 1", hardware.leftDrive1.getCurrentPosition());
//            telemetry.addData("Left motor 2", hardware.leftDrive2.getCurrentPosition());
//            telemetry.addData("Right motor 1", hardware.rightDrive1.getCurrentPosition());
//            telemetry.addData("Right motor 2", hardware.rightDrive2.getCurrentPosition());
//        }
//
//        // Stop motors
//        hardware.linearDrive((float)0.0);
//
//        // Set run mode to RUN_WITHOUT_ENCODER
//        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }

    public void driveEncoderCounts(double power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts, false);

        // Set run mode to RUN_TO_POSITION
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers to the specified amount
        hardware.linearDrive((float)power);

        // Run while op mode is active and motors are busy
        while(opModeIsActive() && hardware.driveMotorsBusy());

        // Stop motors
        hardware.linearDrive((float)0.0);

        // Set run mode to RUN_WITHOUT_ENCODER
        hardware.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveInches(float power, float inches) {
        driveEncoderCounts(power, (int)(inches * hardware.COUNTS_PER_INCH));
    }

    public String vuMarkTelemetry(RelicRecoveryVuMark mark) {
        switch(mark) {
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



    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}