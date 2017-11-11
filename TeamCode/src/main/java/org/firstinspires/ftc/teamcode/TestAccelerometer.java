package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/**
 * Created by SwampbotsAdmin on 11/5/2017.
 */

@Autonomous(name = "Accelerometer Test", group = "Testing")
public class TestAccelerometer extends LinearOpMode {

    TileRunnerREV hardware = new TileRunnerREV();

    BNO055IMU imu;

    Acceleration acceleration;

    AngularVelocity velocity;

    public void runOpMode() {

        telemetry.addLine("Do not press play! Initializing hardware...");
        telemetry.update();

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

        telemetry.addLine("Finished.");
        telemetry.addLine("Press the play button to start.");

        waitForStart();




    }
}
