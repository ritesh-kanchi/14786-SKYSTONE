package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;
import org.firstinspires.ftc.teamcode.Movement;

@Autonomous(name = "Testbench - RF")
@Disabled
public class Testbench extends LinearOpMode {
  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);

  ElapsedTime moveTimer = new ElapsedTime();

  double normPow = 0.6;
  double distPow = 0.75;
  double armPow = 0.75;
  double strafePow = 0.7;
  long gapTime = 150;

  @Override
  public void runOpMode() {

    robot.init(hardwareMap, telemetry, true);
    robot.clawServo(false); // close the claw within the robot
    waitForStart();

    move.right(.7, 5, 0,1); // works the best with what we want it to do
    // right (.7, 5); // goes backwards and ends with a tilt
    // driveSidewayRight(.7, 0, 250); // goes backwards a lot but ends with the proper heading
  }

  public void redFullPath(
          String skystone, double strafeInches, double distanceToSecond, double outtakeDist) {
    telemetry.addData("Skystone Position", skystone); // sends skystone position to telemetry
    telemetry.update(); // update the telemetry record
    if (skystone.equals("Right")) { // specialized entry for right skystone
      move.arm(armPow, 200); // raise the arm at armPow for 175ms
      sleep(gapTime); // gapTime gap
      move.encoderStrafe(strafePow, strafeInches, 1.0); // strafe to strafeInches at 70%
      sleep(gapTime); // gapTime gap
    } else { // nonspecialized entry for anything but skystone
      move.encoderStrafe(strafePow, strafeInches, 1.0); // strafe to strafeInches at 70%
      sleep(gapTime); // gapTime gap
      move.arm(armPow, 200); // raise the arm at armPow for 175ms
      sleep(gapTime); // gapTime gap
    }
    robot.clawServo(true); // close the claw within the robot
    move.forward(normPow, 22, 0); // drive forward 22in at normPow
    move.newTurn(35); // turn to 35deg (left)
    sleep(gapTime); // gapTime gap
    robot.intake("in"); // turn on the "in"
    move.forward(normPow, 8, 35); // drive forward 7in at normPow at 35deg
    robot.intake("stop"); // turn on the "stop"
    move.backward(normPow, 8, 35); // drive backward 8in at normPow at 35deg
    sleep(gapTime); // gapTime gap
    move.newTurn(92); // turn to 90deg, backDist facing building zone
    sleep(250); // 250ms gap
    move.distance1(
            "backwards",
            distPow,
            1250,
            92,
            381,
            robot
                    .backDist); // run distance1 function that drives backwards at 70% for 1250ms at a 90deg
    // until 381mm
    move.newTurn(180); // turn to 180deg, backDist facing foundation
    move.distance(
            "backwards",
            0.2,
            0,
            180,
            5,
            robot.backDist); // run distance function that drives backwards at 20% at a 180deg until 5mm
    robot.foundationServo(false); // close foundation grabbers
    sleep(500); // wait 500ms to complete foundation grab
    move.leftRight(0.6, 20, 20, 1); // drive forwards at 50% for 20in
    sleep(gapTime); // gapTime gap
    move.leftRight(
            1, 55, -55, 1.5); // make a turn at 100% power that pushes foundation horizontally
    robot.foundationServo(true); // open foundation grabbers
    move.forwardT(-0.8, 400); // drive backwards at 80% for 375ms
    sleep(125); // 125ms gap
    move.forwardT(0.6, 400); // drive forwards at 60% for 275ms
    move.newTurn(-85); // turn to -85deg
    robot.intake("out", 0.25); // turn on the "out"
    sleep(650); // 650ms gap
    robot.intake("stop"); // turn on the "stop"
    sleep(gapTime); // gapTime gap
    move.distance1(
            "backwards",
            distPow,
            1250,
            -85,
            distanceToSecond,
            robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
    // -85deg
    // until distanceToSecond
    move.newTurn(35); // turn to 35deg
    robot.intake("in"); // turn on the "in"
    move.forward(normPow, 8, 35); // drive forwards at normPow for 8in at 35deg
    robot.intake("stop"); // turn on the "stop"
    move.backward(normPow, 8, 35); // drive backwards at normPow for 8in at 35deg
    move.newTurn(-84); // turn to -85deg
    move.forward(normPow, outtakeDist, -84); // drive forwards at normPow for outtakeDist at -85deg
    sleep(gapTime); // gapTime gap
    robot.intake("out", 0.25); // turn on the "out"
    sleep(650); // 650ms gap
    robot.intake("stop"); // turn on the "stop"
    move.backward(normPow, 20, -84); // drive backward at normPow for 20in at -85deg
    robot.stop(); // set all motor power to 0, stopping robot
    sleep(500000); // push into sleep so crashes don't occur
  }
}
