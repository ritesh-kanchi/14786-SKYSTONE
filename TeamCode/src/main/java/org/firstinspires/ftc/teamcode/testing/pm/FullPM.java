package org.firstinspires.ftc.teamcode.pm;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.league1.HardwareRobot;

@Autonomous(name = "FullPM")
public class FullPM extends LinearOpMode {
  HardwareRobot robot = new HardwareRobot();
  double imuZ = 0;
  int turns = 0;

  public void runOpMode() {
    robot.init(hardwareMap);
    robot.initGyro(hardwareMap);

    while (opModeIsActive()) {
      Orientation angles = robot.imu.getAngularOrientation();
      telemetry.addData("imu", angles.toString());
      telemetry.update();

      displayStatus();
      moveYB(0.4, 0);
      driveYWT(0.4, 0, 1);

      double bearing = 60;
      double heading = angles.firstAngle;
      moveYB(0.4, (bearing-heading) * 0.02);
    }
  }

  public void moveYB(double forward, double turn) {
    robot.leftFront.setPower(forward + turn);
    robot.rightFront.setPower(forward - turn);
  }

  public void driveYWT(double forward, double turn, double sec) {
    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    while (opModeIsActive()) {
      if (timer.seconds() > sec) break;
      moveYB(forward, turn);
    }
  }

  public void displayStatus() {
    telemetry.addData("LF POS", robot.leftFront.getCurrentPosition());
    telemetry.addData("RF POS", robot.leftFront.getCurrentPosition());

    telemetry.update();
  }

  public void driveYBDT(double forward, double bearing, int dist, double sec) {
    int lfstart = robot.leftFront.getCurrentPosition();
    int rfstart = robot.rightFront.getCurrentPosition();
    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    while (opModeIsActive()) {
      if (timer.seconds() > sec) break;
      int d =
          Math.abs(robot.leftFront.getCurrentPosition() - lfstart)
              + Math.abs(robot.rightFront.getCurrentPosition());
      if (d > dist) break;
      moveYB(forward, bearing);
    }
  }

  public void driveYBT(double forward, double bearing, double sec) {
      ElapsedTime timer = new ElapsedTime();
      timer.reset();
      while (opModeIsActive()) {
          if (timer.seconds() > sec) break;
          moveYB(forward,bearing);
      }
  }

    public void moveYWX(double forward, double turn, double strafe) {
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);
    }
}
