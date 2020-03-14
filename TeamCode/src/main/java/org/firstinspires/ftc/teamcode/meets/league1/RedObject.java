package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "RedObject")
// @Disabled
public class RedObject extends LinearOpMode {

  // tensorflow variables
  private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
  private static final String LABEL_FIRST_ELEMENT = "Stone";
  private static final String LABEL_SECOND_ELEMENT = "Skystone";
  private static final String VUFORIA_KEY =
          "AUr5y1L/////AAABmZwtXPLEokdqmpvjoAOS6MB8SkkFDogwx5jLEHq1NBAyrQprhROpBL1OgAIQbh3ivXpo57uJyLC+hdylicAr3mG+R+Po6RDTiHjtcQy99led05d98Tk6ZhD47Z+cJPikuif4cDQ3KH7/rXwz/3Cjs77AtcDWqYqGvY+9z7GgGO2v9ryjjhL4dPEz51pHS4f57VySPJNTN7qF4xnC8MEnT5A0uqs9mdK0My++lCzO7ezAbElHRVaitR7GlnJy11F3Be/DqFAxuvfT/gNTf9UX2ahE1iR45LE2X6pwAl+cbEiDPFisJ/UYwz8+3A7+zCvOpJbAQDNFdZh6GfqOi8dSabBKvUC9FogeMwP9IRAibz3u";
  // robot specific objects
  HardwareRobot robot = new HardwareRobot();
  ElapsedTime runtime = new ElapsedTime();

  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;

  private double lastZ;
  private int turns;

  private ElapsedTime headingTimer = new ElapsedTime();
  private ElapsedTime backupTimer = new ElapsedTime();

  @Override
  public void runOpMode() {
    // init robot hardware mapping
    robot.init(hardwareMap);

    //  init gyro and imu
    robot.initGyro(hardwareMap);
    heading();

    // bring both servos up from current position
    robot.upLeft();
    robot.upRight();

    // give drive brake zero power behavior
    robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    telemetry.addData("Status", "Resetting Encoders");
    telemetry.update();

    // init robot encoders and calibration
    robot.initEncoders();

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
    telemetry.addData(
            "Path0",
            "Starting at %7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition());
    telemetry.update();
    telemetry.addData("Autonomous Status", "Initialized");
    telemetry.update();
    waitForStart();
    backupTimer.reset();
    

    // scanning loop
    if (opModeIsActive()) {
      while (opModeIsActive()) {

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
                if (100 <= midH && midH <= 160 && 50 <= midV && midV <= 170) {
                  telemetry.addData("SkyStone", "Left");
                  // deactivate tfod for power saving
                  tfod.deactivate();

                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15, 1.0);
                  sleep(100);
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 15, 1.0);
                  gap();
                  encoderStrafe(.8, -130, 1.0);
                  encoderStrafe(.8, -20, 1.0);
                  gap();
                  robot.upLeft();
                    encoderDrive(.3,5,1.0);
                   gap();
                  encoderStrafe(.8, 200, 1.0);
                  encoderStrafe(.8, 30, 1.0);
                  encoderStrafe(.3, 50, 1.0);
                  //gap();
                  encoderDrive(0.3, 40, 1.0);
                  gap();
                  encoderStrafe(.3, -3, 1.0);
                  gap();
                  encoderDrive(.3, 6, 1.0);
                  sleep(250);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15.5, 1.0);
                  sleep(100);
                  robot.downLeft();
                  sleep(50);
                  encoderStrafe(.6, -3, 1.0);
                  sleep(100);
                  encoderDrive(0.3, 18, 1.0);
                  gap();
                  encoderStrafe(0.7, -170, 1.0);
                  setHeading();
                  encoderDrive(0.5, -5.5, 1.0);
                  setHeading();
                  encoderStrafe(0.7, -40, 1.0);
                  setHeading();
                  robot.upLeft();
                  encoderStrafe(0.4, 15, 1.0);
                  gap();
                  // encoderDrive(0.4, -5, 1.0);
                } else if (225 <= midH && midH <= 375 && 50 <= midV && midV <= 170) {
                  telemetry.addData("SkyStone", "Center");
                  // deactivate tfod for power saving
                  tfod.deactivate();
                  encoderStrafe(0.3, -8.5, 1.0);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15, 1.0);
                  sleep(100);
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 15, 1.0);
                  gap();
                  encoderStrafe(.8, -130, 1.0);
                  gap();
                  robot.upLeft();
                    encoderDrive(.3,5,1.0);
                   gap();
                  encoderStrafe(.8, 200, 1.0);
                  encoderStrafe(.8, 30, 1.0);
                  encoderStrafe(.3, 50, 1.0);
                //  gap();
                 
                  encoderDrive(0.3, 40, 1.0);
                  gap();
                  encoderStrafe(.3, -10.25, 1.0);
                  gap();
                  encoderDrive(.3, 3, 1.0);
                  sleep(250);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15.5, 1.0);
                  sleep(100);
                  robot.downLeft();
                  sleep(50);
                  encoderStrafe(.6, -3, 1.0);
                  sleep(100);
                  encoderDrive(0.3, 18, 1.0);
                  gap();
                  encoderStrafe(0.7, -170, 1.0);
                  setHeading();
                  encoderDrive(.3, -5.5, 1.0);
                  setHeading();
                  encoderStrafe(0.7, -30, 1.0);
                  setHeading();
                  robot.upLeft();
                  encoderStrafe(0.4, 15, 1.0);
                  gap();

                } else if (400 <= midH && midH <= 550 && 50 <= midV && midV <= 170) {
                  telemetry.addData("SkyStone", "Right");
                  // deactivate tfod for power saving
                  tfod.deactivate();
                  encoderStrafe(0.3, -5, 1.0);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15, 1.0);
                  sleep(100);
                  robot.downRight();
                  gap();
                  encoderDrive(0.3, 15, 1.0);
                  gap();
                  encoderStrafe(.8, -130, 1.0);
                  gap();
                  robot.upRight();
                  encoderDrive(.3,5,1.0);
                  gap();
                  encoderStrafe(.8, 200, 1.0);
                  encoderStrafe(.8, 20, 1.0);
                  encoderStrafe(.3, 50, 1.0);
                  //  gap();
                  encoderDrive(0.3, 40, 1.0);
                  gap();
                  encoderStrafe(.3, -6.6, 1.0);
                  gap();
                  encoderDrive(.3, 3, 1.0);
                  sleep(250);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15.5, 1.0);
                  sleep(100);
                  robot.downRight();
                  sleep(50);
                  encoderStrafe(.6, -3, 1.0);
                  sleep(100);
                  encoderDrive(0.3, 18, 1.0);
                  gap();
                  encoderStrafe(0.7, -170, 1.0);
                  setHeading();
                  encoderDrive(.3, -4.5, 1.0);
                  setHeading();
                  encoderStrafe(0.7, -30, 1.0);
                  setHeading();
                  robot.upRight();
                  encoderStrafe(0.4, 15, 1.0);
                  gap();
                }
               
              }
               else if (backupTimer.seconds() >  1.5 )
                {
                telemetry.addData("Can't find object", "Running Left");
                telemetry.update();
                    // deactivate tfod for power saving
                  tfod.deactivate();

                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15, 1.0);
                  sleep(100);
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 15, 1.0);
                  gap();
                  encoderStrafe(.8, -130, 1.0);
                  encoderStrafe(.8, -20, 1.0);
                  gap();
                  robot.upLeft();
                   encoderDrive(.3,5,1.0);
                   gap();
                  encoderStrafe(.8, 200, 1.0);
                  encoderStrafe(.8, 30, 1.0);
                  encoderStrafe(.3, 50, 1.0);
                  //gap();
                  encoderDrive(0.3, 40, 1.0);
                  gap();
                  encoderStrafe(.3, -4, 1.0);
                  gap();
                  encoderDrive(.3, 6, 1.0);
                  sleep(250);
                  encoderDrive(0.3, -14.25, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15.5, 1.0);
                  sleep(100);
                  robot.downLeft();
                  sleep(50);
                  encoderStrafe(.6, -3, 1.0);
                  sleep(100);
                  encoderDrive(0.3, 18, 1.0);
                  gap();
                  encoderStrafe(0.7, -170, 1.0);
                  setHeading();
                  encoderDrive(0.5, -5.5, 1.0);
                  setHeading();
                  encoderStrafe(0.7, -40, 1.0);
                  setHeading();
                  robot.upLeft();
                  encoderStrafe(0.4, 15, 1.0);
                  gap();
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

  public void setHeading() { // target will be the angle you want to change by,
    robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // front left motor
    robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // front right motor
    robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // back left motor
    robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // back right motor
    headingTimer.reset();
    double rw;
    while (opModeIsActive()) {
      if (headingTimer.seconds() > 0.75) break; // give the robot .75 seconds to correct its heading
      // rw = 0.02*(-1*heading());
      rw = 0.02 * (heading());
      robot.leftFront.setPower(rw); // front left motor
      robot.rightFront.setPower(-rw); // front right motor
      robot.leftBack.setPower(rw); // back left motor
      robot.rightBack.setPower(-rw); // back right motor
    }
    robot.leftFront.setPower(0); // front left motor
    robot.rightFront.setPower(0); // front right motor
    robot.leftBack.setPower(0); // back left motor
    robot.rightBack.setPower(0); // back right motor
  }
  // private int lastZ; private int turns; these two variables should be in the class, this function
  // returns the heading or angle of the robot since the initialization.
  public double heading() {
    Orientation angles =
            robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double Z = angles.firstAngle;
    if (Z > 140 && lastZ < -140) turns--;
    if (Z < -140 && lastZ > 140) turns++;
    lastZ = Z;
    return turns * 360.0 + Z;
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

  // forward/backward with encoders
  public void encoderDrive(double speed, double inches, double timeoutS) {
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // make sure opmode is still active
    if (opModeIsActive()) {

      // calculate new target position
      int newLeftFrontTarget =
              robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newRightFrontTarget =
              robot.rightFront.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newLeftBackTarget =
              robot.leftBack.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newRightBackTarget =
              robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCount);

      // set new position
      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // turn on RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset time and start motion
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      while (opModeIsActive()
              && (runtime.seconds() < timeoutS)
              && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

        // show running telemetry
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
                "Path2",
                "Running at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // stop all motion
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  // strafe with encoders
  public void encoderStrafe(double speed, double inches, double timeoutS) {
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // make sure opmode is still active
    if (opModeIsActive()) {

      // calculate new target position
      int newLeftFrontTarget =
              robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
              robot.rightFront.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newLeftBackTarget =
              robot.leftBack.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newRightBackTarget =
              robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

      // set new target position
      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // turn on RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset time and start motion
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      while (opModeIsActive()
              && (runtime.seconds() < timeoutS)
              && (robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy())) {

        // show running telemetry
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
                "Path2",
                "Running at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // stop all motion
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  // half-second gap function
  public void gap() {
    sleep(200);
    setHeading();
  }
}