package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "ActiveScan", group = "Concept")
// @Disabled
public class ActiveScan extends LinearOpMode {

    /* VARIABLES*/

    // TF Variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    // Vuforia Variables
    private static final String VUFORIA_KEY =
            "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
    Recognition skystone = null;

    /* OBJECTS */
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

        // Show user that they are ready to press start.
        telemetry.addData("Robot", "Ready");
        telemetry.update();

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
                            if (recognition.getLabel() != LABEL_SECOND_ELEMENT && skystone == null) {
                                telemetry.addData("Skystone", skystone);
                                telemetry.update();
                                // Strafe to the right until ranges are met
                            }
                            // Make sure that we see a skystone
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                // Add that recognition of skystone to our skystone object
                                skystone = recognition;
                                // Check that skystone has data in it, otherwise run the above if loop
                                if (skystone != null) {

                                    // Math calculations and easier variable calls
                                    double skystoneWidth = skystone.getWidth(); // Grabs the width of th
                                    double skystoneHeight = skystone.getHeight(); // Grabs the height of the skystone
                                    double skystoneCenter =
                                            skystoneWidth
                                                    / 2; // Grabs the center of the skystone's x coordinate, can be found by
                                    // dividing getLeft+getRight by 2
                                    double skystoneConfidence =
                                            skystone
                                                    .getConfidence(); // Grabs how confident that what the system sees is a
                                    // skystone
                                    double frameCenterX =
                                            skystone.getImageWidth() / 2; // Grabs the center of the screen's x coordinate
                                    double frameCenterY =
                                            skystone.getImageHeight()
                                                    / 2; // Grabs the center of the screen's y coordinate

                                    // Returning the data through telemetry
                                    telemetry.addData("Skystone", skystone); // Returns the skystone object's data
                                    telemetry.addData(
                                            "Skystone Left", skystone.getLeft()); // Returns the skystone left coordinate
                                    telemetry.addData("Skystone Center", skystoneCenter); // Returns skystoneCenter
                                    telemetry.addData("Skystone Width", skystoneWidth); // Returns skystoneWidth
                                    telemetry.addData("Skystone Height", skystoneHeight); // Returns skystoneHeight
                                    telemetry.addData(
                                            "Skystone Confidence", skystoneConfidence); // Returns skystoneConfidence
                                    telemetry.addData("Center of frame - X", frameCenterX); // Returns frameCenterX
                                    telemetry.addData("Center of frame - Y", frameCenterY); // Returns frameCenterY

                                    // Let's check if we are in the correct place for the alignment
                                    /*
                                     * Why 320 and 260?
                                     * -------------------------
                                     * Well, frameCenterX has a value of 340, so we choose a little bit bigger
                                     * due to the strong fluctuation in the system
                                     * -------------------------
                                     */
                                    if (skystone.getLeft()
                                            < 320) { // If we are less than 320, we are in the wrong area, keep going
                                        telemetry.addData("RANGE", "NO");
                                        // Add a right strafe?
                                    } else if (skystone.getLeft()
                                            > 360) { // If we are greater than 360(for some reason), we are in the wrong
                                        // area, keep going
                                        telemetry.addData("RANGE", "NO");
                                        // Add a left strafe?
                                    } else if (skystone.getLeft() > 320
                                            || skystone.getLeft()
                                            < 360) { // If we are within the range, we are in the correct location
                                        telemetry.addData("RANGE", "WORKS");
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
