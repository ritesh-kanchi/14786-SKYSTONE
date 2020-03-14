package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

@Autonomous(name = "A Fresh Start")
// @Disabled
public class BaseAuton extends LinearOpMode {

  // robot specific objects
  HardwareRobot robot = new HardwareRobot();
  ElapsedTime runtime = new ElapsedTime();

  Orientation lastAngles = new Orientation();
  double globalAngle, correction;

  @Override
  public void runOpMode() {
    // init robot hardware mapping
    robot.init(hardwareMap);
   robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 

    // calibrate encoders and init
    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders"); //
    telemetry.update();

    robot.initEncoders();

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

    // send successful encoder reset
    telemetry.addData(
        "Path0",
        "Starting at %7d :%7d :%7d :%7d",
        robot.leftFront.getCurrentPosition(),
        robot.rightFront.getCurrentPosition(),
        robot.leftBack.getCurrentPosition(),
        robot.rightBack.getCurrentPosition());

    // init telemetry and imu status
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

        // correction
        double imuZ = angles.firstAngle;
        //  moveFT(0, (-imuZ) * 0.02);

        encoderDrive(0.3, 24, 24, 5.0); // S1: Forward 24 Inches with 5 Sec timeout
     //   encoderDrive(0.3, -12, -12, 4.0, imuZ); // S3: Reverse 12 Inches with 4 Sec timeout
      }
    }
  }

  // move robot forward and to a given bearing
  public void moveFB(double forward, double bearing, Orientation angles) {
    angles = robot.imu.getAngularOrientation();
    double heading = angles.firstAngle;
   // moveFT(forward, (heading - bearing) * 0.02);
  }

  public void moveFT(double forward) {
  //  robot.leftFront.setPower(forward + turn);
    //robot.rightFront.setPower(forward - turn);
    //robot.leftBack.setPower(forward + turn);
    //robot.rightBack.setPower(forward - turn);
     robot.leftFront.setPower(forward);
    robot.rightFront.setPower(forward);
    robot.leftBack.setPower(forward);
    robot.rightBack.setPower(forward);
  }

  public void encoderDrive(
      double speed, double leftInches, double rightInches, double timeoutS) {
    int newLFTarget, newRFTarget, newLBTarget, newRBTarget;

    // make sure opmode is active
    if (opModeIsActive()) {

      // find new target for each motor
      newLFTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * robot.inchCount);
      newRFTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * robot.inchCount);
      newLBTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * robot.inchCount);
      newRBTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * robot.inchCount);

      // set target
      robot.leftFront.setTargetPosition(newLFTarget);
      robot.rightFront.setTargetPosition(newRBTarget);
      robot.leftBack.setTargetPosition(newLBTarget);
      robot.rightBack.setTargetPosition(newRBTarget);

      // turn on run to position
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset and start motion
      runtime.reset();

      // speed, forward,
     // moveFT(speed, (-imuZ));
       moveFT(speed);

      // keep looping while we are still active, and there is time left, and both motors are
      // running
      while (opModeIsActive()
          && (runtime.seconds() < timeoutS)
          && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

        // telemetry about current path
        telemetry.addData(
            "Path1",
            "Running to %7d :%7d :%7d :%7d",
            newLFTarget,
            newRFTarget,
            newLBTarget,
            newRBTarget);
        telemetry.addData(
            "Path2",
            "Running at %7d :%7d :%7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition(),
            robot.leftBack.getCurrentPosition(),
            robot.rightBack.getCurrentPosition());
        telemetry.update();
      }

      // stop all motion
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // shut off position
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     // sleep(250); // optional pause after each move
    }
  }
}
