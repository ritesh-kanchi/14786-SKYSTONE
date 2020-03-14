package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

// Stores all hardware and core mechanisms
public class ObjectDetection {

    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    // TF Variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String VUFORIA_KEY =
            "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
    public TFObjectDetector tfod;
    LinearOpMode opmode = null;
    // Detection Objects
    private VuforiaLocalizer vuforia;
    private double conValue = 0.6;

    /* CONSTRUCTOR */
    public ObjectDetection(LinearOpMode aopmode) {

        opmode = aopmode;
    }

    public void init() {

        initVuforia();

        // Check if TF can be used
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opmode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate TF
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opmode.hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId =
                opmode
                        .hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier(
                                "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = conValue;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void tfData(int i, Recognition recognition) {
        opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
        opmode.telemetry.addData(
                String.format("  left,top (%d)", i),
                "%.03f , %.03f",
                recognition.getLeft(),
                recognition.getTop());
        opmode.telemetry.addData(
                String.format("  right,bottom (%d)", i),
                "%.03f , %.03f",
                recognition.getRight(),
                recognition.getBottom());
        opmode.telemetry.addData("midX", (recognition.getLeft() + recognition.getRight()) / 2);
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
