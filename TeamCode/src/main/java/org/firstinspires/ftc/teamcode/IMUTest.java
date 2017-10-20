package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by SwampbotsAdmin on 10/20/2017.
 */

@Autonomous(name = "IMU Test", group = "Testing")
public class IMUTest extends LinearOpMode {

    TileRunnerREV hardware = new TileRunnerREV();

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    final int DEGREES = 73;
    int targetAngle;

    public void runOpMode() {
        // IMU parameters
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(IMUParameters);

        hardware.init(hardwareMap);



        // Set up our telemetry dashboard
        composeTelemetry();

//
//        // Start the logging of measured acceleration
//        //hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        while(opModeIsActive()) {
//            // Handle speed modifiers
//            if(gamepad1.left_bumper)        hardware.driverSpeedMod = hardware.FAST;
//            else if(gamepad1.right_bumper)  hardware.driverSpeedMod = hardware.SLOW;
//            else                            hardware.driverSpeedMod = hardware.NORMAL;
//
//            hardware.linearDrive(
//                    (float)(gamepad1.left_stick_y * hardware.driverSpeedMod),
//                    (float)(gamepad2.right_stick_y * hardware.driverSpeedMod));
//
//            telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_bumper)        hardware.driverSpeedMod = hardware.FAST;
            else if(gamepad1.right_bumper)  hardware.driverSpeedMod = hardware.SLOW;
            else                            hardware.driverSpeedMod = hardware.NORMAL;

            hardware.linearDrive(
                    (float)(gamepad1.left_stick_y * hardware.driverSpeedMod),
                    (float)(gamepad1.right_stick_y * hardware.driverSpeedMod));

            if(gamepad1.dpad_left) {

                hardware.leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                telemetry.addLine(String.format("Turning to %s degrees...", DEGREES));
                telemetry.addLine(String.format("Start angle: %s", angles.firstAngle));
                telemetry.update();
                sleep(1500);

                hardware.linearDrive((float)0.4, (float)-0.4);

                while(opModeIsActive() && angles.firstAngle < DEGREES) {
                    telemetry.addLine(String.format("Turning to %s degrees...", DEGREES));
                    telemetry.addLine(String.format("Current angle: %s", angles.firstAngle));
                    telemetry.update();
                }
                hardware.linearDrive(0);

                telemetry.addLine("Finished with turn.");
                telemetry.update();
                sleep(2000);
            } else if(gamepad1.dpad_right) {
                telemetry.addLine(String.format("Turning to %s degrees...", DEGREES));
                telemetry.addLine(String.format("Starting angle: %s", angles.firstAngle));
                telemetry.update();

                hardware.linearDrive((float)-0.5, (float)0.5);

                while(opModeIsActive() && angles.firstAngle > -DEGREES) {
                    telemetry.addLine(String.format("Turning to %s degrees...", DEGREES));
                    telemetry.addLine(String.format("Current angle: %s", angles.firstAngle));
                    telemetry.update();
                }
                hardware.linearDrive(0);

                telemetry.addLine("Finished with turn.");
                telemetry.update();
                sleep(2000);
            } else if(gamepad1.dpad_up) {
                telemetry.addLine(String.format("Turning %s degrees...", DEGREES));
                telemetry.addData("Starting angle", angles.firstAngle);
                telemetry.addData("Target angle", angles);
                telemetry.update();

                targetAngle = (int)(angles.firstAngle + DEGREES);

                hardware.linearDrive((float)-0.5, (float)0.5);

                while( opModeIsActive() && angles.firstAngle < targetAngle) {
                    telemetry.addLine(String.format("Turning %s degrees...", DEGREES));
                    telemetry.addData("Current angle", angles.firstAngle);
                    telemetry.update();
                }

                hardware.linearDrive(0);

                telemetry.addLine("Finished with turn.");
                telemetry.update();
                sleep(2000);
            } else if(gamepad1.dpad_down) {
                telemetry.addLine(String.format("Turning %s degrees...", DEGREES));
                telemetry.addData("Starting angle", angles.firstAngle);
                telemetry.update();

                targetAngle = (int)(angles.firstAngle - DEGREES);

                hardware.linearDrive((float)-0.5, (float)0.5);

                while(opModeIsActive() && angles.firstAngle > targetAngle) {
                    telemetry.addLine(String.format("Turning %s degrees...", DEGREES));
                    telemetry.addData("Current angle", angles.firstAngle);
                    telemetry.update();
                }

                hardware.linearDrive(0);

                telemetry.addLine("Finished with turn.");
                telemetry.update();
                sleep(2000);
            }

            telemetry.update();
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
