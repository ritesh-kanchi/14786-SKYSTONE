package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "RedPathway", group = "Concept")
@Disabled
public class RedPathway extends LinearOpMode {

    /* VARIABLES*/

    // Path Variables
    static final double skystoneAngle = 30;
    static final float skystoneAngleF = 30;
    static final double kinda90 = 90;
    static final float kinda90F = 90;
    static final double bottomBound = 70;
    static final double topBound = 90;
    /*
     * Why 320 and 260?
     * -------------------------
     * Well, frameCenterX has a value of 340, so we choose a little bit bigger
     * due to the strong fluctuation in the system
     * -------------------------
     */
    // TF Variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    // Vuforia Variables
    private static final String VUFORIA_KEY =
            "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
    static String position = " ";
    Recognition skystone = null;
    /* OBJECTS */
    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);
    Pathways path = new Pathways(robot, move, this);
    FullPath fullPath = new FullPath(robot, move, this);
    // TF Variables
    private TFObjectDetector tfod;

    // Vuforia Variables
    private VuforiaLocalizer vuforia;

    // runOpModeLoop
    @Override
    public void runOpMode() {
       
        // Initialize Vuforia
        initVuforia();

        // Initalize TF if the device is compatible
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate if TF is not null
        if (tfod != null) {
            tfod.activate();
        }

        // Create skystone recognition object to give the skystone a modifiable
        Recognition skystone = null;

        // Initalize Robot
        robot.init(hardwareMap, telemetry, true);

        // Waits for user to hit start
        waitForStart();

        // Checks that the op mode is running
        if (opModeIsActive()) {
            // Opens the opModeIsActive loop to run the code
            while (opModeIsActive()) {
                // Makes sure that TF can run
                if (tfod != null) {
                    // Creates a list of updated recognitions that is seen by the camera
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();
                    // Makes sure that updated recognitions is not null
                    if (updatedRecognitions != null) {
                        // Gives current number of objects detected by the TF system
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        // For the recognitions seen
                        for (Recognition recognition : updatedRecognitions) {
                            // If no skystones are seen and the skystone object is null
                            // Make sure that we see a skystone
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                skystone=recognition;
                                telemetry.addData("Skystone", skystone);
                                telemetry.update();
                                // Add that recognition of skystone to our skystone object
                                /* CHECKPOINT POSITIONING */
                                // Need to test the system first with every stone before adding this
                                // Check that skystone has data in it, otherwise run the above if loop
                                boolean run = true;
                                if (skystone != null && run) {
                                    fullPath.getData(skystone);

                                    // Let's check if we are in the correct place for the alignment
                                    if (skystone.getLeft()
                                            < bottomBound) { // If we are less than 320, we are in the wrong area, keep
                                        // going
                                        telemetry.addData("RANGE", "NO");
                                        move.mecDrive(0,0,0.3,false);
                                    } else if (skystone.getLeft()
                                            > topBound) { // If we are greater than 360(for some reason), we are in the
                                        // wrong
                                        // area, keep going
                                        telemetry.addData("RANGE", "NO");
                                      move.mecDrive(0,0,-0.3,false);
                                    } else if (skystone.getLeft() > bottomBound
                                            || skystone.getLeft()
                                            < topBound) { // If we are within the range, we are in the correct
                                        // location
                                        telemetry.addData("RANGE", "WORKS");
                                     robot.setZero();
                                        sleep(10000);
                                          /* // Drive forward to the stone
                                        // move.driveForward(path.normalPower, 22, 0);
                                        robot.intakeDrop(true);
                                        move.driveBackward(0.5, 3, 0);
                                        fullPath.driveDistance(3, 0.7, 0, 0);
                                        getSkystone();
                                        // Drive an additional 20 inches
                                        move.driveForward(0.8, 20, kinda90F);
                                        // Turn to a 45deg
                                        move.turn(kinda90 / 2);
                                        // Drive forward a bit
                                        move.driveForward(path.normalPower, 4, kinda90F / 2);
                                        // Outtake
                                        move.intake("out", 0.3);
                                        // Drive backwards
                                        move.driveBackward(path.normalPower, 4, kinda90F / 2);
                                        // Turn off intake
                                        move.intake("stop");
                                        // Turn back to 90deg
                                        move.turn(kinda90);
                                        // Drive till the tape
                                        fullPath.driveColor("red", -0.8, 0, 0);
                                        // Drive till the wall
                                        move.driveBackward(path.normalPower, 80, kinda90F);
                                        // checkpoint
                                        // Depending on original position of the skystone, we can determine where the
                                        // second one is
                                        if (position.equals("Left")) {
                                            move.driveForward(path.normalPower, 7, kinda90F);
                                        } else if (position.equals("Center")) {
                                            move.driveForward(path.normalPower, 7, kinda90F);
                                        } else {
                                            move.driveForward(path.normalPower, 7, kinda90F);
                                        }
                                        getSkystone();
                                        // Drive an additional 30 inches
                                        move.driveForward(0.8, 30, kinda90F);
                                        // Turn to a 0deg
                                        move.turn(0);
                                        // Start outtake
                                        move.intake("out", 0.3);
                                        // Drive forward
                                        move.driveForward(path.normalPower, 5, 0);
                                        // Grab foundation
                                        robot.foundationServo(true);
                                        // Drive backwards
                                        move.driveBackward(0.8, 40, 0);
                                        // Turn off intake
                                        move.intake("stop");
                                        // Let go of foundation
                                        robot.foundationServo(false);
                                        // Strafe to the left
                                        move.strafeLeft(0.4, 30, 0);
                                        // Drive forwards
                                        move.driveForward(path.normalPower, 22, 0);
                                        // Turn to 90deg
                                        move.turn(kinda90);
                                        // drive to the tape
                                        fullPath.driveColor("red", -0.7, 0, 0); */
                                        sleep(3000);
                                    }
                                    // Push telemetry to DS
                                    telemetry.update();
                                }
                            }
                            // Increment i
                            i++;
                        }
                        // Overall telemetry push
                        telemetry.update();
                    }
                }
            }
        }

        // Turn off TF when we press stop
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void getSkystone() {
        // Turn to angle the stone
        move.turn(-skystoneAngle);
        // Turn on intake
        move.intake("in");
        // Drive forward to get stone
        move.driveForward(0.6, 10, -skystoneAngleF);
        // Turn off intake
        move.intake("stop");
        // Drive backwards
        move.driveBackward(0.6, 10, -skystoneAngleF);
        // Turn to 90
        move.turn(kinda90);
        // Drive until the tape
        fullPath.driveColor("red", 0.8, 0, 0);
    }

    // initVuforia: Initalizes the Vuforia Engine
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // External Camera Support
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // initTfod: Initalizes the TF Engine
    private void initTfod() {
        int tfodMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // TF Minimum Confidence set to 60%
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
