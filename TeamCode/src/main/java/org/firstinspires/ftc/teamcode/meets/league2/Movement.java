package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Movement {
  
      private final double TURN_SCALE_FACTOR = 0.02;
private ElapsedTime timer = new ElapsedTime();

double turnPower = 0;

  /* VARIABLES */
  // Encoder Variables
  static final double forwardInches = 24;
  static final double backwardInches = 24;
  static final double leftInches = 26;
  static final double rightInches = 26;
  // Power Variables
  static final double fastTurnPower = 0.3;
  static final double slowTurnPower = 0.18;
  
  static final double driveClip = 0.6;
  static final double degRange = 1; 
  
//  static final double fastTurnPower = 0.3;
 // static final double slowTurnPower = 0.18;

 // static final double degRange = 1;
  
  
  ElapsedTime runtime = new ElapsedTime();
  /* OBJECTS */
  HardwareBot robot = null;
  LinearOpMode opmode = null;

  /* CONSTRUCTOR */
  public Movement(HardwareBot arobot, LinearOpMode aopmode) {
    robot = arobot;
    opmode = aopmode;
  }

  /* FUNCTIONS */
  // Global Functions
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
  public void mecDrive(double forward, double turn, double strafe) {
    robot.leftFront.setPower(Range.clip(forward + turn + strafe, -driveClip, driveClip));
    robot.rightFront.setPower(Range.clip(forward - turn - strafe, -driveClip, driveClip));
    robot.leftBack.setPower(Range.clip(forward + turn - strafe, -driveClip, driveClip));
    robot.rightBack.setPower(Range.clip(forward - turn + strafe, -driveClip, driveClip));
  }
  
    public void mecDrive2(double forward, double turn, double strafe) {
    robot.leftFront.setPower(forward + turn + strafe);
    robot.rightFront.setPower(forward - turn - strafe);
    robot.leftBack.setPower(forward + turn - strafe);
    robot.rightBack.setPower(forward - turn + strafe);
  }
  
  public void strafe(String option, double power) {
    if (option.equals("left") || option.equals("Left")) {
      mecDrive2(0, 0, -1);
    }

    if (option.equals("right") || option.equals("Right")) {
      mecDrive2(0, 0, 1);
    }
  }

  // Autonomous Functions
  public double checkDirection(float wantedAngle) {
    double correction, globalAngle, gain = 0.01;
    globalAngle = getGlobalAngle();
    if (globalAngle == wantedAngle) correction = 0;
    else correction = -(globalAngle - wantedAngle);
    correction = correction * gain;

    return correction;
  }

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

  public void driveForward(double power, double distance, float wantedAngle) {

    int leftFrontDist;
    int rightFrontDist;
    int leftBackDist;
    int rightBackDist;

    int moveTicks;
    double correction;

    moveTicks = (int) (distance * forwardInches);
    leftFrontDist = robot.leftFront.getCurrentPosition() + moveTicks;
    rightFrontDist = robot.rightFront.getCurrentPosition() + moveTicks;
    leftBackDist = robot.leftBack.getCurrentPosition() + moveTicks;
    rightBackDist = robot.rightBack.getCurrentPosition() + moveTicks;

    positionPower(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

    while (opmode.opModeIsActive()
        && (robot.leftFront.isBusy()
            && robot.rightFront.isBusy()
            && robot.leftBack.isBusy()
            && robot.rightBack.isBusy())) {

      opmode.telemetry.addData("Forward", distance);
      opmode.telemetry.update();

      correction = checkDirection(wantedAngle);

      robot.leftFront.setPower(power - correction);
      robot.rightFront.setPower(power + correction);
      robot.leftBack.setPower(power - correction);
      robot.rightBack.setPower(power + correction);
    }

    robot.setZero();

    robot.setEncoders(true);
    turn((double) wantedAngle);
  }

  public void driveBackward(double power, double distance, float wantedAngle) {

    int leftFrontDist;
    int rightFrontDist;
    int leftBackDist;
    int rightBackDist;

    int moveTicks;
    double correction;

    moveTicks = (int) (distance * backwardInches);
    leftFrontDist = robot.leftFront.getCurrentPosition() - moveTicks;
    rightFrontDist = robot.rightFront.getCurrentPosition() - moveTicks;
    leftBackDist = robot.leftBack.getCurrentPosition() - moveTicks;
    rightBackDist = robot.rightBack.getCurrentPosition() - moveTicks;

    positionPower(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

    while (opmode.opModeIsActive()
        && (robot.leftFront.isBusy()
            && robot.rightFront.isBusy()
            && robot.leftBack.isBusy()
            && robot.rightBack.isBusy())) {

      opmode.telemetry.addData("Backwards", distance);
      opmode.telemetry.update();

      correction = checkDirection(wantedAngle);

      robot.leftFront.setPower(power + correction);
      robot.rightFront.setPower(power - correction);
      robot.leftBack.setPower(power + correction);
      robot.rightBack.setPower(power - correction);
    }

    robot.setZero();

    robot.setEncoders(true);
    turn((double) wantedAngle);
  }

  public void strafeRight(double power, double distance, float wantedAngle) {

    int leftFrontDist;
    int rightFrontDist;
    int leftBackDist;
    int rightBackDist;

    int moveTicks;
    double correction = 0;

    moveTicks = (int) (distance * rightInches);
    leftFrontDist = robot.leftFront.getCurrentPosition() + moveTicks;
    rightFrontDist = robot.rightFront.getCurrentPosition() - moveTicks;
    leftBackDist = robot.leftBack.getCurrentPosition() - moveTicks;
    rightBackDist = robot.rightBack.getCurrentPosition() + moveTicks;

    positionPower(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

    while (opmode.opModeIsActive()
        && (robot.leftFront.isBusy()
            && robot.rightFront.isBusy()
            && robot.leftBack.isBusy()
            && robot.rightBack.isBusy())) {

      opmode.telemetry.addData("Strafe Right", distance);
      opmode.telemetry.update();

      correction = checkDirection(wantedAngle);

      robot.leftBack.setPower(power + correction);
      robot.rightBack.setPower(power + correction);
      robot.leftFront.setPower(power - correction);
      robot.rightFront.setPower(power - correction);
    }

    robot.setZero();

    robot.setEncoders(true);
    turn((double) wantedAngle);
  }

  public void strafeLeft(double power, double distance, float wantedAngle) {

    int leftFrontDist;
    int rightFrontDist;
    int leftBackDist;
    int rightBackDist;

    int moveTicks;
    double correction = 0;

    moveTicks = (int) (distance * leftInches);
    leftFrontDist = robot.leftFront.getCurrentPosition() - moveTicks;
    rightFrontDist = robot.rightFront.getCurrentPosition() + moveTicks;
    leftBackDist = robot.leftBack.getCurrentPosition() + moveTicks;
    rightBackDist = robot.rightBack.getCurrentPosition() - moveTicks;

    positionPower(power, leftFrontDist, rightFrontDist, leftBackDist, rightBackDist);

    while (opmode.opModeIsActive()
        && (robot.leftFront.isBusy()
            && robot.rightFront.isBusy()
            && robot.leftBack.isBusy()
            && robot.rightBack.isBusy())) {

      opmode.telemetry.addData("Strafe Left", distance);
      opmode.telemetry.update();

      correction = checkDirection(wantedAngle);

      robot.leftBack.setPower(power - correction);
      robot.rightBack.setPower(power - correction);
      robot.leftFront.setPower(power + correction);
      robot.rightFront.setPower(power + correction);
    }

    robot.setZero();

    robot.setEncoders(true);
    turn((double) wantedAngle);
  }

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

    robot.setZero();
  }

  public void positionPower(
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

  public void encoderStrafe(double speed, double inches, double timeoutS) {
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

      while (opmode.opModeIsActive()
          && (runtime.seconds() < timeoutS)
          && (robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy())) {

        // show running telemetry
        opmode.telemetry.addData(
            "Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        opmode.telemetry.addData(
            "Path2",
            "Running at %7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition());
        opmode.telemetry.update();
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
public void newTurn(double degrees, double duration, double power) {
  double degreeDiff = degrees - getGlobalAngle();

  timer.reset();
  while(timer.seconds() < duration) {
  opmode.telemetry.addData("DegDiff",degreeDiff);
  opmode.telemetry.update();

  turnPower = degreeDiff*TURN_SCALE_FACTOR*power;

  if (Math.abs(degreeDiff) > 10) {
    robot.leftFront.setPower(-turnPower);
    robot.rightFront.setPower(turnPower);
      robot.leftBack.setPower(-turnPower);
      robot.rightBack.setPower(turnPower);
      }
      degreeDiff = degrees - getGlobalAngle();
      }
robot.setZero();
}

}