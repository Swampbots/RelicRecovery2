package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kawaiiPlat on 7/29/2017.
 */

public class DemoBot {

    // Speed modifier variables
    public final double SLOW     = 0.4;
    public final double NORMAL   = 0.7;
    public final double FAST     = 1.0;
    double driverSpeedMod        = NORMAL;
    double utilitySpeedMod       = NORMAL;

    // Motor objects
    public DcMotor leftDrive1   = null;
    public DcMotor rightDrive1  = null;


    // Sensor Objects
    public ColorSensor colorSensor  = null;


    // Hardware map
    HardwareMap hwMap = null;


    // Elapsed time
    private ElapsedTime elapsedTime = null;


    // Constructor
    public DemoBot() {
    }


    // Hardware map initialization
    public void init(HardwareMap ahwMap) {

        // Get the hardware map
        hwMap = ahwMap;


        // Get the motors
        leftDrive1  = hwMap.dcMotor.get("xmotor1");
        rightDrive1 = hwMap.dcMotor.get("ymotor1");


        // Get the sensors
        colorSensor     = hwMap.colorSensor.get("colorsensor");


        // Set the motor directions
        leftDrive1.setDirection (DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set the motor powers to zero
        leftDrive1  .setPower(0);
        rightDrive1 .setPower(0);


        // Set the motors to not use encoders
        leftDrive1.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)elapsedTime.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        elapsedTime.reset();
    }
}
