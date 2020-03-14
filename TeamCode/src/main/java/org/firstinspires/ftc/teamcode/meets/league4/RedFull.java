package org.firstinspires.ftc.teamcode.meets.league4;

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

// Autonomous with R1-3
@Autonomous(name = "RedFull")
@Disabled
public class RedFull extends LinearOpMode {

    // TF Variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
    /* CREATE VARIABLES */
    // Robot Variables
    double normPow = 0.6;
    double slowPow = 0.4;
    double strafePow = 0.8;
    double armPower = 1;
    String position = "none";
    /* CREATE OBJECTS */
    // Robot Objects
    HardwareBot robot = new HardwareBot();
    MoveTwo move = new MoveTwo(robot, this);

    // Other Objects
    ElapsedTime backupTimer = new ElapsedTime();

    float addedValue = 0;

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
                            robot.tfData(i, recognition);
                            i++;
                            // TF determines positon from given Vuforia data
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                // Skystone mid variables
                                double midH = (recognition.getLeft() + recognition.getRight()) / 2;
                                double midV = (recognition.getTop() + recognition.getBottom()) / 2;

                                // Checks mid vars to be between certain range, returns position
                                if (midH <= 215) {
                                    position = "Left";
                                    redPath(position, -6);
                                } else if (225 <= midH && midH < 400) {
                                    position = "Center";
                                    redPath(position, 6);
                                    sleep(4000);
                                } else if (400 <= midH) {
                                    position = "Right";
                                    redPath(position, 10);
                                }
                            } else if (backupTimer.seconds() > 3.5) {
                                position = "BACKUP";
                                redPath(position, 6);
                                sleep(4000);
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

    public void redPath(String skystone, double strafeInches) {
        telemetry.addData("Skystone Position", skystone);
        telemetry.update();
        if (position.equals("Right")) {
            move.arm(1, 150);
            sleep(100);
            move.encoderStrafe(0.4, strafeInches, 1.0);
            sleep(100);
        } else {
            move.encoderStrafe(0.4, strafeInches, 1.0);
            sleep(100);
            move.arm(1, 150);
            sleep(100);
        }


        move.forward(normPow, 22, 0);
        move.turn(30);
        sleep(100);
        robot.intake("in");
        move.forward(normPow, 7, 30);
        robot.intake("stop");
        move.backward(normPow, 7, 30);
        move.turn(90);
    
    /*
      sleep(100);
      move.distance("backwards", slowPow, 1500, 90, 178, robot.backDist);
    move.turn(180);
      sleep(100);
    move.backward(slowPow, 5, 180);
    robot.foundationServo(true);
      sleep(100);
    move.forward(slowPow, 10, 180);
    move.turn(90);
      sleep(100);
    robot.clawServo(true);
      sleep(100);
    move.arm(armPower, 500);
      robot.clawServo(false);
      sleep(100);
    move.arm(-armPower, 500);
      sleep(100);
    robot.foundationServo(false);
      sleep(100);
    move.forward(0.6, 30, 90); */
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
