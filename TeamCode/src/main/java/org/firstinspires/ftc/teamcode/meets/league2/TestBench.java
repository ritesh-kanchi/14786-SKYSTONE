package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestBench")
@Disabled
public class TestBench extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);
  ElapsedTime stop = new ElapsedTime();

  @Override
  public void runOpMode() {

    robot.init(hardwareMap, telemetry);
    robot.autonInit();
    waitForStart();

    while (opModeIsActive()) {
      stop.reset();
      //  move.turn(90, .5);
        sleep(4000);
      /*  move.driveForward(0.3, 80, 0);
        gap();
        move.turn(88);
         telemetry.addData("Angle",move.getGlobalAngle());
        telemetry.update();
        gap();
        move.driveForward(0.3, 20, 88);
         move.turn(88);
         telemetry.addData("global angle",move.getGlobalAngle());
         telemetry.update();
         move.turn(88);
           telemetry.addData("global angle",move.getGlobalAngle());
         telemetry.update();
        sleep(1000);
         move.turn(88);
           telemetry.addData("global angle",move.getGlobalAngle());
         telemetry.update();
        move.driveBackward(0.3, 20, 88);
        gap();
        move.turn(0);
         telemetry.addData("Angle",move.getGlobalAngle());
        telemetry.update();
        gap();
        move.driveBackward(0.3, 80, 0);
        gap();
      //  move.turn(0);
        telemetry.addData("Angle",move.getGlobalAngle());
        telemetry.update(); */
      while (stop.seconds() < 40) {
          robot.setZero();
      }
    }
  }

  public void gap() {
    sleep(200);
  }
}
