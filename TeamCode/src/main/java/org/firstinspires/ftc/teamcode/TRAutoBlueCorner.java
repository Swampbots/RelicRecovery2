package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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


    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing hardware and Vuforia... do not press play.");
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



        telemetry.addLine("Hardware Initialized.");
        telemetry.addLine("Press the play button to start.");
        telemetry.update();


        waitForStart();


        // Start looking for the vision targets
        relicTrackables.activate();

        // Try to find the vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        //////////////////////
        // JEWEL LOGIC

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

            driveInches((float)0.7, (float)3.0);
        }
        else {
            jewelColor = JewelColor.RED;
            telemetry.addData("Blue", hardware.colorSensor.blue());
            telemetry.addData("Red", hardware.colorSensor.red());
            telemetry.addLine("Left jewel was red. Driving backwards...");
            telemetry.update();
            sleep(1500);

            driveInches((float)0.7, (float)-3.0);
        }

        hardware.jewelServo.setPosition(hardware.ARM_UP);

        // JEWEL LOGIC END
        //////////////////////

        // Figure out which cryptobox key you found
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



        while(opModeIsActive()) {
            telemetry.addLine("Visible vision target:");
            telemetry.addLine(vuMarkTelemetry(vuMark));
            telemetry.addLine();
            telemetry.addData("Jewel Color", jewelColor.toString());
            telemetry.addLine();
            telemetry.addData("Distance to drive", inches);
            telemetry.update();
        }
    }

    public void driveEncoderCounts(double power, int counts) {
        // Set target positions
        hardware.setDriveTargetPosition(counts);

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
}