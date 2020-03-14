package org.firstinspires.ftc.teamcode.league1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "NewAuton")
public class NewAuton extends LinearOpMode {

  ElapsedTime runtime = new ElapsedTime();
  HardwareRobot robot = new HardwareRobot();


  @Override
  public void runOpMode() {

    robot.init(hardwareMap);
    robot.initGyro(hardwareMap);

    waitForStart();
    while (opModeIsActive()) {
      driveYBT(0,90,0,.5);
      //driveYBT(0.5,0,0,.5);
    // strafeLeft(0.3,0,2);
    }

  }
  public void strafeLeft(double forward, double bearing, double sec) {
    ElapsedTime timer = new ElapsedTime();
    double gyroAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
    double correction = (gyroAngle-bearing) / 80.0;
    timer.reset();
    while (opModeIsActive()) {
      if (timer.seconds() > sec) break;
      robot.leftFront.setPower(forward + correction);
      robot.rightFront.setPower(-forward - correction);
      robot.leftBack.setPower(-forward + correction);
      robot.rightBack.setPower(forward - correction);
    }
  }

  public void moveYB(double forward, double turn) {
    robot.leftFront.setPower(forward + turn);
    robot.rightFront.setPower(forward - turn);
    robot.leftBack.setPower(forward + turn);
    robot.rightBack.setPower(forward - turn);
  }


 /* public void strafe(double power, double angle, double timeout) {
    robot.useEncoders();
  robot.mecanum(power,-power,-power,power);
    runtime.reset();
    while (runtime.milliseconds() > timeout) {
      float gyroAngle =
          robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
              .firstAngle;
      telemetry.addData("gyro", gyroAngle);
      telemetry.update();
      double correction = (gyroAngle - angle);
      robot.mecanum(
          -power + correction, power - correction, power + correction, -power - correction);
      sleep(20);
    }
    robot.mecanum(0,0,0,0);
  } */
 public void moveYWX(double forward, double turn, double strafe) {
   robot.leftFront.setPower(forward + turn + strafe);
   robot.rightFront.setPower(forward - turn - strafe);
   robot.leftBack.setPower(forward + turn - strafe);
   robot.rightBack.setPower(forward - turn + strafe);
 }

  // start the robot moving forward (Y) to a given bearing
  public void moveYB(double forward, double bearing, double strafe) {
    Orientation angles = robot.imu.getAngularOrientation();
    double heading = angles.firstAngle;
    moveYWX(forward, (heading-bearing) * 0.01, 0);
  }

  // drive the robot forward (Y) and/or turning (W) for seconds (T)
  public void driveYBT(double forward, double bearing, double strafe, double sec) {
    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    while (opModeIsActive()) {
      if (timer.seconds() > sec) break;
      moveYB(forward, bearing, strafe);
    }

  }
}
