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

// Autonomous with R1-3, containing Foundation
@Autonomous(name = "RedFull")
@Disabled
public class RedFull extends LinearOpMode {

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
                                  path.firstSkystoneRed("Left", -5, path.acrossFieldRight + 1 + 5);
                                  outtakeFoundationRed();
                                  secondSkystoneRed((path.acrossFieldRight * 1.45) + 5, 70 + 5);
                                  outtakeFoundationRed();
                                  parkRed();
                              } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 170) {
                                  path.firstSkystoneRed("Center", 6, path.acrossFieldRight - 5 + 25);
                                  outtakeFoundationRed();
                                  secondSkystoneRed((path.acrossFieldRight * 1.45) - 6 + 25, 64 + 25);
                                  outtakeFoundationRed();
                                  parkRed();
                              } else if (400 <= midH && 50 <= midV && midV <= 170) {
                                  path.firstSkystoneRed("Right", 14, path.acrossFieldRight - 13 + 25);
                                  outtakeFoundationRed();
                                  secondSkystoneRed((path.acrossFieldRight * 1.45) - 14 + 25, 58 + 25);
                                  outtakeFoundationRed();
                                  parkRed();
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

    public void secondSkystoneRed(double acrossDist, double backDist) {
        move.driveBackward(path.normalPower, backDist, -89);
        path.gap();
        // Turn to corner of Second Skystone
        move.turn(path.angleToStoneRed - 2);
        // Turn on Intake
        move.intake("in");
        path.intakeTime();
        // Drive forward to get Skystone
        move.driveForward(path.normalPower, path.forBack + 5, path.angleToStoneRedF - 2);
        // Turn off Intake
        move.intake("stop");
        // Back up
        move.driveBackward(path.normalPower, path.forBack, path.angleToStoneRedF - 2);
        path.gap();
        // Turn to 90ish
        move.turn(-88);
        path.gap();
        // Drive across tape
        move.driveForward(path.normalPower, acrossDist, -88);
        // Park
        path.gap();
    }

    public void parkRed() {
        // Drive back onto the tape
        move.driveBackward(0.7, path.parkDist, -88);
        // Strafe to the side to enable larger robot to fit besides us
        move.encoderStrafe(path.strafePower, -4, 1.0);
        // Avoid code looping
        sleep(40000);
    }

    public void outtakeFoundationRed() {
        move.turn(-45);
        move.driveForward(.85, 10, -45);
        move.intake("out", 0.4);
        sleep(400);
        move.intake("stop");
        move.driveBackward(.85, 10, 0);
        path.gap();
        move.turn(-88);
        move.turn(-89);
        path.gap();
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
      tfodParameters.minimumConfidence = 0.5;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }
}