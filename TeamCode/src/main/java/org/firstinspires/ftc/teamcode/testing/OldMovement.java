package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareBot;

public class OldMovement {

  static final double strafeLeftInches = 75.8;
  static final double strafeRightInches = 76.8;
  static final double forwardInches = 69.4;
  static final double backwardInches = 70.3;

  static final double slowDownPower = 0;
  static final double normalPower = 0;
  static final double turnPower = 0.2;
  static final double strafePower = 0;

  HardwareBot robot = null;
  LinearOpMode opmode = null;

  public ElapsedTime moveTime = new ElapsedTime();

  public OldMovement(HardwareBot bot, LinearOpMode aopmode) {
    robot = bot;
    opmode = aopmode;
  }

  private double checkDirection(double angle) {
    double correction, globalAngle, gain = 1;

    globalAngle = getAngle();

    if (globalAngle == angle) {
      correction = 0;
    } // no adjustment.
    else {
      correction = -(globalAngle - angle);
    }

    correction = correction * gain;

    return correction;
  }

  public double getAngle() {
    Orientation currentAngle =
            robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = currentAngle.firstAngle - robot.lastAngles.firstAngle;

    if (deltaAngle < -180) deltaAngle += 360;
    else if (deltaAngle > 180) deltaAngle -= 360;

    robot.globalAngle += deltaAngle;

    robot.lastAngles = currentAngle;

    return robot.globalAngle;
  }

  public void strafeLeft(double power, double distance, float angle) {

    robot.setEncoders(true);
    double correction = 0;

    int moveTicks = (int) (distance * forwardInches);

    if (opmode.opModeIsActive()) {


      int lfdist = robot.leftFront.getCurrentPosition() - moveTicks;
      int rfdist = robot.rightFront.getCurrentPosition() + moveTicks;
      int lbdist = robot.leftBack.getCurrentPosition() + moveTicks;
      int rbdist = robot.rightBack.getCurrentPosition() - moveTicks;

      encoderPower(power,lfdist,rfdist,lbdist,rbdist);

      while (opmode.opModeIsActive() && robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy()) {

        opmode.telemetry.addData("LF",robot.leftFront.getCurrentPosition());
        opmode.telemetry.addData("RF",robot.rightFront.getCurrentPosition());
        opmode.telemetry.addData("LB",robot.leftBack.getCurrentPosition());
        opmode.telemetry.addData("RB",robot.rightBack.getCurrentPosition());

        int newlfdist = moveTicks - robot.leftFront.getCurrentPosition();
        int newrfdist = moveTicks - robot.rightFront.getCurrentPosition();
        int newlbdist = moveTicks - robot.leftBack.getCurrentPosition();
        int newrbdist = moveTicks - robot.rightBack.getCurrentPosition();

        robot.leftFront.setTargetPosition(newlfdist);
        robot.rightFront.setTargetPosition(newrfdist);
        robot.leftBack.setTargetPosition(newlbdist);
        robot.rightBack.setTargetPosition(newrbdist);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opmode.telemetry.update();

        correction = checkDirection(angle);

        robot.leftFront.setPower(power - correction);
        robot.rightFront.setPower(power - correction);
        robot.leftBack.setPower(power + correction);
        robot.rightBack.setPower(power + correction);
      }
      robot.setZero();
      robot.setEncoders(true);
    }
  }

  public void strafeRight(double power, double distance, float angle, double moveSecs) {

    robot.setEncoders(true);
    double correction = 0;

    int moveTicks = (int) (distance * forwardInches);

    if (opmode.opModeIsActive()) {


      int lfdist = robot.leftFront.getCurrentPosition() + moveTicks;
      int rfdist = robot.rightFront.getCurrentPosition() - moveTicks;
      int lbdist = robot.leftBack.getCurrentPosition() - moveTicks;
      int rbdist = robot.rightBack.getCurrentPosition() + moveTicks;

      encoderPower(power,lfdist,rfdist,lbdist,rbdist);

      while (opmode.opModeIsActive() && robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy()) {

        opmode.telemetry.addData("LF",robot.leftFront.getCurrentPosition());
        opmode.telemetry.addData("RF",robot.rightFront.getCurrentPosition());
        opmode.telemetry.addData("LB",robot.leftBack.getCurrentPosition());
        opmode.telemetry.addData("RB",robot.rightBack.getCurrentPosition());

        int newlfdist = moveTicks - robot.leftFront.getCurrentPosition();
        int newrfdist = moveTicks - robot.rightFront.getCurrentPosition();
        int newlbdist = moveTicks - robot.leftBack.getCurrentPosition();
        int newrbdist = moveTicks - robot.rightBack.getCurrentPosition();

        robot.leftFront.setTargetPosition(newlfdist);
        robot.rightFront.setTargetPosition(newrfdist);
        robot.leftBack.setTargetPosition(newlbdist);
        robot.rightBack.setTargetPosition(newrbdist);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opmode.telemetry.update();

        correction = checkDirection(angle);

        robot.leftFront.setPower(power + correction);
        robot.rightFront.setPower(power + correction);
        robot.leftBack.setPower(power - correction);
        robot.rightBack.setPower(power - correction);
      }
      robot.setZero();
      robot.setEncoders(true);
    }
  }

  public void driveForward(double power, double distance, double angle) {

    robot.setEncoders(true);
    double correction = 0;

    int moveTicks = (int) (distance * forwardInches);

    if (opmode.opModeIsActive()) {


      int lfdist = robot.leftFront.getCurrentPosition() + moveTicks;
      int rfdist = robot.rightFront.getCurrentPosition() + moveTicks;
      int lbdist = robot.leftBack.getCurrentPosition() + moveTicks;
      int rbdist = robot.rightBack.getCurrentPosition() + moveTicks;

      encoderPower(power,lfdist,rfdist,lbdist,rbdist);

      while (opmode.opModeIsActive() && robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy()) {

        opmode.telemetry.addData("LF",robot.leftFront.getCurrentPosition());
        opmode.telemetry.addData("RF",robot.rightFront.getCurrentPosition());
        opmode.telemetry.addData("LB",robot.leftBack.getCurrentPosition());
        opmode.telemetry.addData("RB",robot.rightBack.getCurrentPosition());

        int newlfdist = moveTicks - robot.leftFront.getCurrentPosition();
        int newrfdist = moveTicks - robot.rightFront.getCurrentPosition();
        int newlbdist = moveTicks - robot.leftBack.getCurrentPosition();
        int newrbdist = moveTicks - robot.rightBack.getCurrentPosition();

        robot.leftFront.setTargetPosition(newlfdist);
        robot.rightFront.setTargetPosition(newrfdist);
        robot.leftBack.setTargetPosition(newlbdist);
        robot.rightBack.setTargetPosition(newrbdist);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opmode.telemetry.update();

        correction = checkDirection(angle);

        robot.leftFront.setPower(power - correction);
        robot.rightFront.setPower(power + correction);
        robot.leftBack.setPower(power - correction);
        robot.rightBack.setPower(power + correction);
      }
      robot.setZero();
      robot.setEncoders(true);
    }
  }

  public void driveBackward(double power, double distance, float angle, double moveSecs) {

    robot.setEncoders(true);
    double correction = 0;

    int moveTicks = (int) (distance * forwardInches);

    if (opmode.opModeIsActive()) {


      int lfdist = robot.leftFront.getCurrentPosition() - moveTicks;
      int rfdist = robot.rightFront.getCurrentPosition() - moveTicks;
      int lbdist = robot.leftBack.getCurrentPosition() - moveTicks;
      int rbdist = robot.rightBack.getCurrentPosition() - moveTicks;

      encoderPower(power,lfdist,rfdist,lbdist,rbdist);

      while (opmode.opModeIsActive() && robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy()) {

        opmode.telemetry.addData("LF",robot.leftFront.getCurrentPosition());
        opmode.telemetry.addData("RF",robot.rightFront.getCurrentPosition());
        opmode.telemetry.addData("LB",robot.leftBack.getCurrentPosition());
        opmode.telemetry.addData("RB",robot.rightBack.getCurrentPosition());

        int newlfdist = moveTicks - robot.leftFront.getCurrentPosition();
        int newrfdist = moveTicks - robot.rightFront.getCurrentPosition();
        int newlbdist = moveTicks - robot.leftBack.getCurrentPosition();
        int newrbdist = moveTicks - robot.rightBack.getCurrentPosition();

        robot.leftFront.setTargetPosition(newlfdist);
        robot.rightFront.setTargetPosition(newrfdist);
        robot.leftBack.setTargetPosition(newlbdist);
        robot.rightBack.setTargetPosition(newrbdist);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opmode.telemetry.update();

        correction = checkDirection(angle);

        robot.leftFront.setPower(power + correction);
        robot.rightFront.setPower(power - correction);
        robot.leftBack.setPower(power + correction);
        robot.rightBack.setPower(power - correction);
      }
      robot.setZero();
      robot.setEncoders(true);
    }
  }

  public void turn(double degree) {

    double degreeCalc = degree - getAngle();

    if (degreeCalc > 0) { // Left turn
      robot.leftFront.setPower(-turnPower);
      robot.rightFront.setPower(turnPower);
      robot.leftBack.setPower(-turnPower);
      robot.rightBack.setPower(turnPower);
    } else { // Right turn
      robot.leftFront.setPower(turnPower);
      robot.rightFront.setPower(-turnPower);
      robot.leftBack.setPower(turnPower);
      robot.rightBack.setPower(-turnPower);
    }

    degreeCalc = degree - getAngle();

    robot.setZero();
  }

  public void moveYB(double bearing){
    robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    Orientation angles = robot.imu.getAngularOrientation();
    double heading = angles.firstAngle;
    double turn = (heading-bearing)*0.02;
    robot.leftFront.setPower(0 + turn);
    robot.rightFront.setPower(0 - turn);
    robot.leftBack.setPower(0 + turn);
    robot.rightBack.setPower(0 - turn);

    if (getAngle() == bearing) {
      robot.setZero();
      robot.setBrake(false);
    }
  }

  public void encoderPower(double power, int lfdist, int rfdist, int lbdist, int rbdist) {

    robot.leftFront.setTargetPosition(lfdist);
    robot.rightFront.setTargetPosition(rfdist);
    robot.leftBack.setTargetPosition(lbdist);
    robot.rightBack.setTargetPosition(rbdist);


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
}
