package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MoveTwo {

    static final double fastTurnPower = 0.35;
    static final double slowTurnPower = 0.15;
    static final double degRange = 1;
    double slowPower = 0.3;
    /* VARIABLES */
    double forwardTicks = 31.2;
    double backwardTicks = 31.2;
    double leftTicks = 40.3;
    double rightTicks = 40.3;
    long stopTime = 100;

    double TURN_SCALE_FACTOR = 0.01;

    /* OBJECTS */
    HardwareBot robot = null;
    LinearOpMode opmode = null;
    ElapsedTime turnTimer = new ElapsedTime();
    ElapsedTime gyroTimer = new ElapsedTime();
    ElapsedTime motionTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    /* CONSTRUCTOR */
    public MoveTwo(HardwareBot arobot, LinearOpMode aopmode) {
        robot = arobot;
        opmode = aopmode;
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

        if (opmode.opModeIsActive()) {

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

            // how long the motors should run
            while (opmode.opModeIsActive()
                    && (robot.leftFront.isBusy()
                    && robot.rightFront.isBusy()
                    && robot.leftBack.isBusy()
                    && robot.rightBack.isBusy())) {

                opmode.telemetry.addData("Path1", "Running to %7d :%7d", leftFrontDist, rightFrontDist);
                opmode.telemetry.addData(
                        "Path2",
                        "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());

                // find how much we should correct by
                correction = checkDirection(wantedAngle);

                // corrections
                robot.leftFront.setPower(power - correction);
                robot.rightFront.setPower(power + correction);
                robot.leftBack.setPower(power - correction);
                robot.rightBack.setPower(power + correction);
            }

            // stop the robot
            robot.stop();

            // turn off run to position by setting to RUN_USING_ENCODERS
            robot.stop();
            robot.setEncoders(true);
            turn((double) wantedAngle);
            opmode.sleep(stopTime);
        }
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

            // how long the motors should run
            while (opmode.opModeIsActive()
                    && (robot.leftFront.isBusy()
                    && robot.rightFront.isBusy()
                    && robot.leftBack.isBusy()
                    && robot.rightBack.isBusy())) {

                opmode.telemetry.addData("Path1", "Running to %7d :%7d", leftFrontDist, rightFrontDist);
                opmode.telemetry.addData(
                        "Path2",
                        "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());

                // find how much we should correct by
                correction = checkDirection(wantedAngle);

                // corrections
                robot.leftFront.setPower(power + correction);
                robot.rightFront.setPower(power - correction);
                robot.leftBack.setPower(power + correction);
                robot.rightBack.setPower(power - correction);
            }

            // turn off run to position by setting to RUN_USING_ENCODERS
            robot.stop();
            robot.setEncoders(true);
            turn((double) wantedAngle);
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

            // how long the motors should run
            while (opmode.opModeIsActive() && (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                opmode.telemetry.addData("Path1", "Running to %7d :%7d", leftFrontDist, rightFrontDist);
                opmode.telemetry.addData(
                        "Path2",
                        "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());

                // find how much we should correct by
                correction = checkDirection(wantedAngle);

                // corrections
                robot.leftFront.setPower(power + correction);
                robot.rightFront.setPower(power - correction);
                robot.leftBack.setPower(power - correction);
                robot.rightBack.setPower(power + correction);
            }

            robot.setEncoders(true);
            // turn((double) wantedAngle);
            robot.stop();
            opmode.sleep(stopTime);
        }
    }

    // strafe right using encoders while maintaining a heading
    public void right(double power, double distance, float wantedAngle) {

        int leftFrontDist, rightFrontDist, leftBackDist, rightBackDist;
        double correction;

        if (opmode.opModeIsActive()) {

            int moveTicks = (int) (distance * rightTicks);

            // DOES THIS DO ANYTHING

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDist = robot.leftFront.getCurrentPosition() + moveTicks;
            rightFrontDist = robot.rightFront.getCurrentPosition() - moveTicks;
            leftBackDist = robot.leftBack.getCurrentPosition() - moveTicks;
            rightBackDist = robot.rightBack.getCurrentPosition() + moveTicks;

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

            // how long the motors should run
            while (opmode.opModeIsActive() && (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                opmode.telemetry.addData("Path1", "Running to %7d :%7d", leftFrontDist, rightFrontDist);
                opmode.telemetry.addData(
                        "Path2",
                        "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());

                // find how much we should correct by
                correction = checkDirection(wantedAngle);

                // corrections
                robot.leftFront.setPower(power - correction);
                robot.rightFront.setPower(power + correction);
                robot.leftBack.setPower(power + correction);
                robot.rightBack.setPower(power - correction);
            }

            robot.setEncoders(true);
            //   turn((double) wantedAngle);
            robot.stop();
            opmode.sleep(stopTime);
        }
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

    public void distance(
            String direction,
            double power,
            double time,
            float angle,
            double mmValue,
            DistanceSensor distSensor) {
        forGyro(-power, angle, time);
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
}
