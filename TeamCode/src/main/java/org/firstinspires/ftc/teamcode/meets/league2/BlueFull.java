package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Autonomous with B1-3, containing Foundation
@Autonomous(name = "BlueFull")
@Disabled
public class BlueFull extends LinearOpMode {

  private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
  private static final String LABEL_FIRST_ELEMENT = "Stone";
  private static final String LABEL_SECOND_ELEMENT = "Skystone";
  private static final String VUFORIA_KEY =
      "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";

  /* CREATE OBJECTS */
  // Robot Objects
  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);
  Pathways path = new Pathways(robot, move, this);
  ElapsedTime runtime = new ElapsedTime();

  // Detection Objects
  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;

  @Override
  public void runOpMode() {
    // Init Vuforia
    initVuforia();

    // Check if TF can be used
    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod();
    } else {
      telemetry.addData("Sorry!", "This device is not compatible with TFOD");
    }

    // Activate TF
    if (tfod != null) {
      tfod.activate();
    }

    // Robot initialization
    robot.init(hardwareMap, telemetry);
    robot.autonInit();

    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (tfod != null) {
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
              // Data from TF+Vuforia
              telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
              telemetry.addData(
                  String.format("  left,top (%d)", i),
                  "%.03f , %.03f",
                  recognition.getLeft(),
                  recognition.getTop());
              telemetry.addData(
                  String.format("  right,bottom (%d)", i),
                  "%.03f , %.03f",
                  recognition.getRight(),
                  recognition.getBottom());
              i++;
              // TF determines positon from given Vuforia data
              if (recognition.getLabel() == "Skystone") {
                // Skystone mid variables
                double midH = (recognition.getLeft() + recognition.getRight()) / 2;
                double midV = (recognition.getTop() + recognition.getBottom()) / 2;

                // Checks mid vars to be between certain range, returns position
                if (midH <= 200 && 50 <= midV && midV <= 170) {
                  // Get's the first Skystone
                  path.firstSkystoneBlue("Left", -6, path.acrossFieldLeft + 1);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeBlue(70);
                  // Get's the second Skystone
                  path.secondSkystoneBlue(path.acrossFieldLeft * 1.45);
                  // Park
                  path.parkBlue();
                } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 170) {
                  // Get's the first Skystone
                  path.firstSkystoneBlue("Center", 6, path.acrossFieldLeft - 5);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeBlue(64);
                  // Get's the second Skystone
                  path.secondSkystoneBlue((path.acrossFieldLeft * 1.45) - 6);
                  // Park
                  path.parkBlue();
                } else if (400 <= midH && 50 <= midV && midV <= 170) {
                  // Get's the first Skystone
                  path.firstSkystoneBlue("Right", 14, path.acrossFieldLeft - 13);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeBlue(58);
                  // Get's the second Skystone
                  path.secondSkystoneBlue((path.acrossFieldLeft * 1.45) - 14);
                  // Park
                  path.parkBlue();
                }
              }
            }
            telemetry.update();
          }
        }
      }
    }

    if (tfod != null) {
      tfod.shutdown();
    }
  }

  // Init Vuforia Function
  private void initVuforia() {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }
  // Init TF Function
  private void initTfod() {
    int tfodMonitorViewId =
        hardwareMap
            .appContext
            .getResources()
            .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minimumConfidence = 0.6;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }
}