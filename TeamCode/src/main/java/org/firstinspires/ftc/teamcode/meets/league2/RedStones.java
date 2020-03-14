package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Autonomous with R1-3
@Autonomous(name = "RedStones")
public class RedStones extends LinearOpMode {

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
  ElapsedTime backupTimer = new ElapsedTime();

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
    backupTimer.reset();

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
                if (midH <= 215 && 50 <= midV && midV <= 170) {
                  // Get's the first Skystone
                  path.firstSkystoneRed("Left", -5, path.acrossFieldRight+1);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeRed(70);
                  // Get's the second Skystone
                  path.secondSkystoneRed((path.acrossFieldRight * 1.45));
                  // Park
                  path.parkRed();
                } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 170) {
                  // Get's the first Skystone
                  path.firstSkystoneRed("Center", 8, path.acrossFieldRight - 5);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeRed(62);
                  // Get's the second Skystone
                  path.secondSkystoneRed((path.acrossFieldRight * 1.45) - 6);
                  // Park
                  path.parkRed();
                } else if (400 <= midH && 50 <= midV && midV <= 170) {
                // Get's the first Skystone
                   // Return location of the Skystone
    telemetry.addData("SkyStone", "Right");
    telemetry.update();
    // Lower Intake
    // Strafe to get corner of First Skystone
    move.encoderStrafe(path.strafePower, 22, 1.0);
    sleep(100);
    move.encoderStrafe(path.strafePower, 6, 1.0);
    robot.drawbridge(1, 3.25);
    // Drive Forward
    move.driveForward(path.normalPower, path.goToStone, 0);
    path.gap();
    // Turn to corner of Skystone
    move.turn(path.angleToStoneRed - 3);
    // Turn on Intake
    move.intake("in");
    sleep(250);
    // Drive forward to get Skystone
    move.driveForward(path.normalPower, path.forBack,path.angleToStoneRedF - 3);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(path.normalPower, path.forBack-3, path.angleToStoneRedF - 3);
    path.gap();
    // Turn to 90ish
    move.turn(-88);
    path.gap();
    // Drive across tape
    move.driveForward(path.normalPower, path.acrossFieldRight - 13, -88);
                  // Outtake's the first Skystone and goes to get second
                  path.outtakeRed(56);
                  // Get's the second Skystone
                  path.secondSkystoneRed((path.acrossFieldRight * 1.45) - 14);
                  // Park
                  path.parkRed(); 
                }
              }
              else if (backupTimer.seconds() > 3.5) {
                // RUN CENTER PROGRAM
                // Get's the first Skystone
                path.firstSkystoneRed("BACKUP-Center", 8, path.acrossFieldRight - 5);
                // Outtake's the first Skystone and goes to get second
                path.outtakeRed(62);
                // Get's the second Skystone
                path.secondSkystoneRed((path.acrossFieldRight * 1.45) - 6);
                // Park
                path.parkRed();
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