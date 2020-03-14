package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

@Autonomous(name = "A Fresh Object Start")
// @Disabled
public class BaseObject extends LinearOpMode {

  // tensorflow variables
  private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
  private static final String LABEL_FIRST_ELEMENT = "Stone";
  private static final String LABEL_SECOND_ELEMENT = "Skystone";
  private static final String VUFORIA_KEY =
      "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
  // robot specific objects
  HardwareRobot robot = new HardwareRobot();
  ElapsedTime runtime = new ElapsedTime();

  Orientation lastAngles = new Orientation();
  double globalAngle, correction;
  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;

  @Override
  public void runOpMode() {
    // init robot hardware mapping
    robot.init(hardwareMap);

    // init gyro params
    robot.initGyro(hardwareMap);
    // calibrate the gyro/imu
    telemetry.addData("Mode", "calibrating...");
    telemetry.update();
    // wait till imu is calibrated
    while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
      sleep(50);
      idle();
    }

    // vuforia camera init
    initVuforia();

    // checks for tensorflow compatibility and inits
    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod();
    } else {
      telemetry.addData("Sorry!", "This device is not compatible with TFOD");
    }

    if (tfod != null) {
      tfod.activate();
    }

    telemetry.addData("Autonomous Status", "Initialized");
    telemetry.addData("IMU Calibration Status", robot.imu.getCalibrationStatus().toString());
    telemetry.update();

    waitForStart();

    // scanning loop
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // display IMU values
        Orientation angles = robot.imu.getAngularOrientation();
        telemetry.addData("IMU", angles.toString());
        telemetry.update();

        if (tfod != null) {
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());

            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
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
              // tf determines positon from given vuforia data
              if (recognition.getLabel() == "Skystone") {
                // skystone mid variables
                double midH = (recognition.getLeft() + recognition.getRight()) / 2;
                double midV = (recognition.getTop() + recognition.getBottom()) / 2;

                // checks mid vars to be between certain range, returns position
                if (100 <= midH && midH <= 150 && 50 <= midV && midV <= 150) {
                  telemetry.addData("SkyStone", "Left");

                } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 150) {
                  telemetry.addData("SkyStone", "Center");

                } else if (400 <= midH && midH <= 550 && 50 <= midV && midV <= 150) {
                  telemetry.addData("SkyStone", "Right");
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

  // init vuforia function
  private void initVuforia() {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }

  // init tf function
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
