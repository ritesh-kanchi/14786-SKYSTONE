package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Movement {

  static final double fastTurnPower = 0.35;
  static final double driveClip = 0.6;
  static final double slowTurnPower = 0.15;
  static final double degRange = 1;
    static final double distOvershoot = 25;

    static final double newFastTurnPower = 0.4;
    static final double newSlowTurnPower = 0.2;

    double slowPower = 0.3;

  /* VARIABLES */
  double forwardTicks = 31.2;
  double backwardTicks = 31.2;
  double leftTicks = 40.3;
  double rightTicks = 40.3;
  long stopTime = 100;

  /* OBJECTS */
  HardwareBot robot = null;
  LinearOpMode opmode = null;
  ElapsedTime gyroTimer = new ElapsedTime();
  ElapsedTime motionTimer = new ElapsedTime();
  ElapsedTime runtime = new ElapsedTime();

  /* CONSTRUCTOR */
  public Movement(HardwareBot arobot, LinearOpMode aopmode) {
    robot = arobot;
    opmode = aopmode;
  }

  /* FUNCTIONS */
  // Global Functions
  // intake direction by typing "in" or "out" and giving it a power
  public void intake(String option, double value) {
    double power = value;
    if (option.equals("in") || option.equals("In") || option.equals("IN")) {
      robot.leftIntake.setPower(-power);
      robot.rightIntake.setPower(-power);
    } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
      robot.leftIntake.setPower(power);
      robot.rightIntake.setPower(power);
    } else {
      robot.leftIntake.setPower(0);
      robot.rightIntake.setPower(0);
    }
  }

  // overrides previous intake function and gives power of 1
  public void intake(String option) {
    double power = 1;
    if (option.equals("in") || option.equals("In") || option.equals("IN")) {
      robot.leftIntake.setPower(-power);
      robot.rightIntake.setPower(-power);
    } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
      robot.leftIntake.setPower(power);
      robot.rightIntake.setPower(power);
    } else {
      robot.leftIntake.setPower(0);
      robot.rightIntake.setPower(0);
    }
  }

  // TeleOp Functions
  // function to move the chassis during TeleOp

  public void mecDrive(double forward, double turn, double strafe, double clip) {
      if (clip != 0 && clip != 1) {
      robot.leftFront.setPower(Range.clip(forward + turn + strafe, -driveClip, driveClip));
      robot.rightFront.setPower(Range.clip(forward - turn - strafe, -driveClip, driveClip));
      robot.leftBack.setPower(Range.clip(forward + turn - strafe, -driveClip, driveClip));
      robot.rightBack.setPower(Range.clip(forward - turn + strafe, -driveClip, driveClip));
    } else {
      robot.leftFront.setPower(forward + turn + strafe);
      robot.rightFront.setPower(forward - turn - strafe);
      robot.leftBack.setPower(forward + turn - strafe);
      robot.rightBack.setPower(forward - turn + strafe);
    }
  }

  // strafe function that uses mecDrive to strafe during TeleOp
  public void strafe(String option, double power) {
    if (option.equals("left") || option.equals("Left")) {
        mecDrive(0, 0, -power, 0);
    }

    if (option.equals("right") || option.equals("Right")) {
        mecDrive(0, 0, power, 0);
    }
  }

  /* FUNCTIONS - GLOBAL */
  public double checkDirection(float wantedAngle) {
    double correction, globalAngle, gain = 0.01; // lets mess with this gain value
    globalAngle = getGlobalAngle();
    if (globalAngle == wantedAngle) correction = 0;
    else correction = -(globalAngle - wantedAngle);
    correction = correction * gain;

    return correction;
  }

  // returns given angle
  public double getGlobalAngle() {
    Orientation currentAngles =
            robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = currentAngles.firstAngle - robot.lastAngles.firstAngle;

    if (deltaAngle < -180) deltaAngle += 360;
    else if (deltaAngle > 180) deltaAngle -= 360;

    robot.globalAngle += deltaAngle;

    robot.lastAngles = currentAngles;

    return robot.globalAngle;
  }

  /* FUNCTIONS - AUTONOMOUS */
  // drive forward using encoders while maintaining a heading
  public void forward(double power, double distance, float wantedAngle) {

    int leftFrontDist, rightFrontDist, leftBackDist, rightBackDist;
    double correction;

      // if (opmode.opModeIsActive()) {

      int moveTicks = (int) (distance * forwardTicks);

      // DOES THIS DO ANYTHING

    /*   robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */

      leftFrontDist = robot.leftFront.getCurrentPosition() + moveTicks;
      rightFrontDist = robot.rightFront.getCurrentPosition() + moveTicks;
      leftBackDist = robot.leftBack.getCurrentPosition() + moveTicks;
      rightBackDist = robot.rightBack.getCurrentPosition() + moveTicks;

      setTarget(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

      // how long the motors should run
      while (opmode.opModeIsActive()
              && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

          encoderData(leftFrontDist, rightFrontDist);
          opmode.telemetry.update();

          // find how much we should correct by
          correction = checkDirection(wantedAngle);

          // corrections
          robot.leftFront.setPower(power - correction);
          robot.rightFront.setPower(power + correction);
          robot.leftBack.setPower(power - correction);
          robot.rightBack.setPower(power + correction);
      }

      robot.stop();
      robot.setEncoders(true);
      opmode.sleep(stopTime);
      //  }
  }

  // drive backward using encoders while maintaining a heading
  public void backward(double power, double distance, float wantedAngle) {

    int leftFrontDist, rightFrontDist, leftBackDist, rightBackDist;
    double correction;

    if (opmode.opModeIsActive()) {

      int moveTicks = (int) (distance * backwardTicks);

      // DOES THIS DO ANYTHING

      /*   robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */

      leftFrontDist = robot.leftFront.getCurrentPosition() - moveTicks;
      rightFrontDist = robot.rightFront.getCurrentPosition() - moveTicks;
      leftBackDist = robot.leftBack.getCurrentPosition() - moveTicks;
      rightBackDist = robot.rightBack.getCurrentPosition() - moveTicks;

        setTarget(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

      // how long the motors should run
      while (opmode.opModeIsActive()
              && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

          encoderData(leftFrontDist, rightFrontDist);

        // find how much we should correct by
        correction = checkDirection(wantedAngle);

        // corrections
        robot.leftFront.setPower(power + correction);
        robot.rightFront.setPower(power - correction);
        robot.leftBack.setPower(power + correction);
        robot.rightBack.setPower(power - correction);
      }

      robot.stop();
      robot.setEncoders(true);
      opmode.sleep(stopTime);
    }
  }

  // strafe left using encoders while maintaining a heading
  public void left(double power, double distance, float wantedAngle) {

    int leftFrontDist, rightFrontDist, leftBackDist, rightBackDist;
    double correction;

    if (opmode.opModeIsActive()) {

      int moveTicks = (int) (distance * leftTicks);

      // DOES THIS DO ANYTHING

      robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      leftFrontDist = robot.leftFront.getCurrentPosition() - moveTicks;
      rightFrontDist = robot.rightFront.getCurrentPosition() + moveTicks;
      leftBackDist = robot.leftBack.getCurrentPosition() + moveTicks;
      rightBackDist = robot.rightBack.getCurrentPosition() - moveTicks;

        setTarget(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

      // how long the motors should run
      while (opmode.opModeIsActive() && (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

          encoderData(leftFrontDist, rightFrontDist);

        // find how much we should correct by
        correction = checkDirection(wantedAngle);

        // corrections
        robot.leftFront.setPower(power + correction);
          robot.rightFront.setPower(power + correction);
        robot.leftBack.setPower(power - correction);
          robot.rightBack.setPower(power - correction);
      }

      robot.stop();
        robot.setEncoders(true);
      opmode.sleep(stopTime);
    }
  }

  // strafe right using encoders while maintaining a heading
  public void right(double speed, double distance, float angleRobotToFace, double time) {

      int leftFrontDist;
      int rightFrontDist;
      int leftBackDist;
      int rightBackDist;

      int moveCounts;
      double correction = 0;
      moveCounts = (int) (distance * rightTicks);
      leftFrontDist = robot.leftFront.getCurrentPosition() + moveCounts;
      rightFrontDist = robot.rightFront.getCurrentPosition() - moveCounts;
      leftBackDist = robot.leftBack.getCurrentPosition() - moveCounts;
      rightBackDist = robot.rightBack.getCurrentPosition() + moveCounts;

      setTarget(speed, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);
      motionTimer.reset();
      while (motionTimer.seconds() < time
              && robot.rightBack.isBusy()
              && robot.leftFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightFront.isBusy()) {

          encoderData(leftFrontDist, rightFrontDist);
          correction = checkDirection(angleRobotToFace);

          robot.leftBack.setPower(speed + correction);
          robot.rightBack.setPower(speed + correction);
          robot.leftFront.setPower(speed - correction);
          robot.rightFront.setPower(speed - correction);
      }

      robot.stop();
      robot.setEncoders(true);
      opmode.sleep(stopTime);
  }

  // turns to a given heading
  public void turn(double degrees) {

    double degreeDiff = degrees - getGlobalAngle();
    // rotate until turn is completed.
    while (Math.abs(degreeDiff) > degRange) {
      opmode.telemetry.addData("DegDiff", degreeDiff);
      opmode.telemetry.update();
      if (Math.abs(degreeDiff) > 10) { // Fast turn
        if (degreeDiff > 0) { // Left turn
          robot.leftFront.setPower(-fastTurnPower);
          robot.rightFront.setPower(fastTurnPower);
          robot.leftBack.setPower(-fastTurnPower);
          robot.rightBack.setPower(fastTurnPower);
        } else { // Right turn
          robot.leftFront.setPower(fastTurnPower);
          robot.rightFront.setPower(-fastTurnPower);
          robot.leftBack.setPower(fastTurnPower);
          robot.rightBack.setPower(-fastTurnPower);
        }
      } else { // Slow turn
        if (degreeDiff > 0) { // Left turn
          robot.leftFront.setPower(-slowTurnPower);
          robot.rightFront.setPower(slowTurnPower);
          robot.leftBack.setPower(-slowTurnPower);
          robot.rightBack.setPower(slowTurnPower);
        } else { // Right turn
          robot.leftFront.setPower(slowTurnPower);
          robot.rightFront.setPower(-slowTurnPower);
          robot.leftBack.setPower(slowTurnPower);
          robot.rightBack.setPower(-slowTurnPower);
        }
      }

      degreeDiff = degrees - getGlobalAngle();
    }

    robot.stop();
  }

    public void newTurn(double degrees) {
        double degreeDiff = degrees - getGlobalAngle();
        // rotate until turn is completed.
        while (Math.abs(degreeDiff) > 0.5) {
            opmode.telemetry.addData("DegDiff", degreeDiff);
            opmode.telemetry.update();
            if (Math.abs(degreeDiff) > 20) { // Fast turn
                if (degreeDiff > 0) { // Left turn
                    robot.leftFront.setPower(-newFastTurnPower);
                    robot.rightFront.setPower(newFastTurnPower);
                    robot.leftBack.setPower(-newFastTurnPower);
                    robot.rightBack.setPower(newFastTurnPower);
                } else { // Right turn
                    robot.leftFront.setPower(newFastTurnPower);
                    robot.rightFront.setPower(-newFastTurnPower);
                    robot.leftBack.setPower(newFastTurnPower);
                    robot.rightBack.setPower(-newFastTurnPower);
                }
            } else { // Slow turn
                if (degreeDiff > 0) { // Left turn
                    robot.leftFront.setPower(-newSlowTurnPower);
                    robot.rightFront.setPower(newSlowTurnPower);
                    robot.leftBack.setPower(-newSlowTurnPower);
                    robot.rightBack.setPower(newSlowTurnPower);
                } else { // Right turn
                    robot.leftFront.setPower(newSlowTurnPower);
                    robot.rightFront.setPower(-newSlowTurnPower);
                    robot.leftBack.setPower(newSlowTurnPower);
                    robot.rightBack.setPower(-newSlowTurnPower);
                }
            }

            degreeDiff = degrees - getGlobalAngle();
        }

        robot.stop();
    }

    public void leftRight(double power, double leftDist, double rightDist, double seconds) {

    int leftFrontDist, rightFrontDist, leftBackDist, rightBackDist;
    double correction;

        // if (opmode.opModeIsActive()) {

        int moveLeftTicks = (int) (leftDist * forwardTicks);
        int moveRightTicks = (int) (rightDist * forwardTicks);

        // DOES THIS DO ANYTHING

    /*   robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */

        leftFrontDist = robot.leftFront.getCurrentPosition() + moveLeftTicks;
        rightFrontDist = robot.rightFront.getCurrentPosition() + moveRightTicks;
        leftBackDist = robot.leftBack.getCurrentPosition() + moveLeftTicks;
        rightBackDist = robot.rightBack.getCurrentPosition() + moveRightTicks;

        setTarget(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);
        motionTimer.reset();
        // how long the motors should run
        while (opmode.opModeIsActive()
                && (motionTimer.seconds() < seconds)
                && (robot.leftFront.isBusy()
                && robot.rightFront.isBusy()
                && robot.leftBack.isBusy()
                && robot.rightBack.isBusy())) {

            encoderData(leftFrontDist, rightFrontDist);
            opmode.telemetry.update();

            // find how much we should correct by

            // corrections
            robot.leftFront.setPower(power);
            robot.rightFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightBack.setPower(power);
        }

        robot.stop();
        robot.setEncoders(true);
        opmode.sleep(stopTime);
        //  }
  }

  // used to go forwards at a certain heading and uses time
  public void forGyro(double power, float wantedAngle, double msec) {
    double correction;
    gyroTimer.reset();
    while (gyroTimer.milliseconds() < msec) {
      correction = checkDirection(wantedAngle);

      robot.leftFront.setPower(power - correction);
      robot.rightFront.setPower(power + correction);
      robot.leftBack.setPower(power - correction);
      robot.rightBack.setPower(power + correction);
    }
    robot.stop();
  }

  // overloaded for usage without time
  public void forGyro(double power, float wantedAngle) {
    double correction = checkDirection(wantedAngle);
    robot.leftFront.setPower(power - correction);
    robot.rightFront.setPower(power + correction);
    robot.leftBack.setPower(power - correction);
    robot.rightBack.setPower(power + correction);
  }

    public void forwardT(double power, double msec) {
        motionTimer.reset();
    while (motionTimer.milliseconds() < msec) {
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
    }
        robot.stop();
  }

  public void distance(
          String direction,
          double power,
          double time,
          float angle,
          double mmValue,
          DistanceSensor distSensor) {
      if (direction.equals("forwards") || direction.equals("Forwards")) {
          power = power;
      } else {
          power *= -1;
      }
      forGyro(power, angle, time);

    opmode.sleep(50);

      while (distSensor.getDistance(DistanceUnit.MM) > mmValue + distOvershoot) {

          opmode.telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.MM));
          opmode.telemetry.addData("Power", Math.abs(power));

          forGyro(power, angle);
          if (Math.abs(power) > 0.2) {
              if (direction.equals("forwards") || direction.equals("Forwards")) {
                  power -= 0.01;
              } else {
                  power += 0.01;
              }
      }
    }

      opmode.telemetry.addData("Final Distance", distSensor.getDistance(DistanceUnit.MM));
      opmode.telemetry.update();
    robot.stop();
  }

  public void distance1(
          String direction,
          double power,
          double time,
          float angle,
          double mmValue,
          DistanceSensor distSensor) {
      if (direction.equals("forwards") || direction.equals("Forwards")) {
          power = power;
      } else {
          power *= -1;
      }
      forGyro(power, angle, time);

    opmode.sleep(50);

      while (distSensor.getDistance(DistanceUnit.MM) > mmValue + 25) {

          opmode.telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.MM));
          opmode.telemetry.addData("Power", slowPower);

          forGyro(-slowPower, angle);
          if (slowPower > 0.2) {
              slowPower = slowPower - 0.01;
          }
      }
      robot.stop();

      opmode.telemetry.addData("Final Distance", distSensor.getDistance(DistanceUnit.MM));
      opmode.telemetry.update();
      robot.stop();
  }

  public void arm(double power, double msec) {
    opmode.telemetry.addLine("Arm");
    opmode.telemetry.update();
    motionTimer.reset();
    while (motionTimer.milliseconds() < msec) {
      robot.chainMotor.setPower(power);
    }
    robot.stop();
    robot.chainMotor.setPower(0);
  }

    public void encoderStrafe(double power, double inches, double timeoutS) {

    // make sure opmode is still active
    if (opmode.opModeIsActive()) {

      // calculate new target position
      int newLeftFrontTarget =
          robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
          robot.rightFront.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newLeftBackTarget =
          robot.leftBack.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newRightBackTarget =
          robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

        setTarget(
                power, newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);

      while (opmode.opModeIsActive()
          && (runtime.seconds() < timeoutS)
          && (robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy())) {

        // show running telemetry
          encoderData(newLeftFrontTarget, newRightFrontTarget);
      }

        robot.stop();
        robot.setEncoders(true);
        opmode.sleep(stopTime);
    }
    }

    public void setTarget(
            double power, int leftFrontDist, int rightFrontDist, int leftBackDist, int rightBackDist) {
        robot.leftFront.setTargetPosition(leftFrontDist);
        robot.rightFront.setTargetPosition(rightFrontDist);
        robot.leftBack.setTargetPosition(leftBackDist);
        robot.rightBack.setTargetPosition(rightBackDist);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Math.abs(power);
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
    }

    public void encoderData(int leftFrontDist, int rightFrontDist) {
        opmode.telemetry.addData("Wanted Values", "Running to %7d :%7d", leftFrontDist, rightFrontDist);
        opmode.telemetry.addData(
                "Current Values",
                "Running at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        opmode.telemetry.update();
  }
}
