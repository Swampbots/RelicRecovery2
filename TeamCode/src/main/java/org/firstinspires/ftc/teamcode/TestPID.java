package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by SwampbotsAdmin on 10/22/2017.
 */

//@Disabled
@Autonomous(name = "PID test", group = "Testing")
//@Disabled
public class TestPID extends LinearOpMode {

    // IMU object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    TileRunnerREV hardware = new TileRunnerREV();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("DO NOT PRESS PLAY! Initializing hardware.");
        telemetry.update();

        hardware.init(hardwareMap);

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

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Turning to heading of -90...");
        telemetry.update();

        turnToHeadingPID(-90);

        telemetry.addLine("Successful.");
        telemetry.addData("Heading", heading());
        telemetry.update();
        sleep(2000);

        telemetry.addLine("Turning to heading of 180...");
        telemetry.update();

        turnToHeadingPID(180);

        while(opModeIsActive()) {
            telemetry.addLine("Finished");
            telemetry.addData("Heading", heading());
            telemetry.update();
        }

    }





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
        while(angle > 180) {
            angle -= 360;
        }
        while(angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double heading() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
