package org.firstinspires.ftc.teamcode.meets.league3;

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
@Autonomous(name = "RedFull")
@Disabled
public class RedFull extends LinearOpMode {
  
  ElapsedTime gyroTimer = new ElapsedTime();
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
  ElapsedTime ftimer = new ElapsedTime();

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
    robot.init(hardwareMap, telemetry, true);

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
                tfData(i, recognition);
              i++;
              // TF determines positon from given Vuforia data
              if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                // Skystone mid variables
                double midH = (recognition.getLeft() + recognition.getRight()) / 2;
                  double midV =
                          (recognition.getTop() + recognition.getBottom())
                                  / 2; // do we need to pump this back in to track better

                // Checks mid vars to be between certain range, returns position
                if (midH <= 215) {
                    path.firstSkystoneRed("Left", -5, path.acrossFieldRight + 8); // added 7
                    // Outtake's the first Skystone and goes to get second
                    //skystone45Red(79);
                    skystone90Red(89);
                    path.secondSkystoneRed((path.acrossFieldRight * 1.45) + 35);
                    skystoneFoundationRed();

                }
                /* MIGHT NEED TO CHANGE A FEW VALUES, JUST SET UP FORMAT */

                else if (225 <= midH && midH <= 375) {
                    path.firstSkystoneRed("Center", 7, path.acrossFieldRight + 5);
                    // Outtake's the first Skystone and goes to get second
                    //skystone45Red(71);
                    skystone90Red(90);
                    path.secondSkystoneRed((path.acrossFieldRight * 1.45) + 29);
                    skystoneFoundationRed();

                    /*move.encoderStrafe(0.4, -20, 1.0); // DOES THIS ALTERNATE PER PATH???
                    move.encoderStrafe(0.4, -20, 1.0); // SAME HERE - IT SHOULDNT, KEEP SAME POSITION EVERY TIME

                    skystoneRedPark();*/

                } else if (400 <= midH) {
                    path.firstSkystoneRed("Right", 15, (path.rightSkystone1+18)*1.1); // hard coded strafeInches, this value doesnt do anything
                    // Outtake's the first Skystone and goes to get second
                    //skystone45Red(65);
                    skystone90Red((path.rightSkystone1+29)*1.1);
                    path.secondSkystoneRed((path.acrossFieldRight * 1.45) + 45);
                    skystoneFoundationRed();

                    /*move.encoderStrafe(0.4, -20, 1.0); // DOES THIS ALTERNATE PER PATH???
                    move.encoderStrafe(0.4, -20, 1.0); // SAME HERE - IT SHOULDNT, KEEP SAME POSITION EVERY TIME

                    skystoneRedPark();*/
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

    public void tfData(int i, Recognition recognition) {
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
    }

    public void skystoneRedPark() {
      move.turn(0);
        move.driveForward(path.normalPower, 18, 0);
        // move.turn(-89); // if we are not strafing
        move.turn(3);
        // move.driveBackward(path.normalPower, 15, -89); if we are not strafing
        move.strafeLeft(0.4, 16, 3);
        sleep(40000);
    }

    public void skystoneFoundationRed() {
        move.turn(0);
        move.gyroTime(0.3, 0, 750, gyroTimer);
        move.turn(0);
        robot.setZero();
        sleep(250);
        move.gyroTime(0.3, 0, 500, gyroTimer);
        ftimer.reset();
        robot.foundationServo(false);
        // can we change this to a 250 to match the bottom
        robot.setZero();
        sleep(750);
        move.gyroTime(-0.5, 0, 2000, gyroTimer);
        robot.foundationServo(true);
        sleep(250);
        move.intake("out", 0.4);
        sleep(300);
        move.intake("stop");
        move.turn(3);
        /*move.encoderStrafe(0.4, -20, 1.0);
        move.encoderStrafe(0.4, -19, 1.0);*/
        move.strafeLeft(0.4, 35, 3);
        robot.setZero();

        skystoneRedPark();
        
    }

    public void skystone45Red(double backDist) {
        move.turn(-45);

        sleep(100);
        move.driveForward(path.normalPower, 10, -45);
        move.intake("out", 0.55); // can we bump this up to .8
        sleep(500); // can we reduce this to 300
        move.driveBackward(path.normalPower, 14, -45);
        move.intake("stop");
        move.turn(-89);

        move.driveBackward(path.normalPower, backDist, -89);
  }
    
    public void skystone90Red(double backDist) {
        move.turn(3);

        sleep(100);
        move.driveForward(path.normalPower, 9, 3);
        move.intake("out", 0.45); // can we bump this up to .8
        sleep(500); // can we reduce this to 300
        move.driveBackward(path.normalPower, 9, 3);
        move.intake("stop");
        // move.turn(-89); // if we are not strafing
        move.turn(3);
        robot.setZero();
        sleep(250);
        // move.driveBackward(path.normalPower, backDist, -89); // if we are not strafing\
        move.strafeLeft(0.4, backDist, 3);
        robot.setZero();
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
