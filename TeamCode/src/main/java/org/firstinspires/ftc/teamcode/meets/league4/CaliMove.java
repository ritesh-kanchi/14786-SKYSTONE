package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "CaliMove")
@Disabled
public class CaliMove extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    MoveTwo move = new MoveTwo(robot, this);

    double normPow = 0.6;
    double slowPow = 0.4;
    double strafePow = 0.8;
    double armPower = 1;
    String position = "none";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, true);
        waitForStart();
        move.distance("backwards", 0.7, 1650, 0, 229 * 3, robot.backDist);
        move.turn(125);


        telemetry.addData("Foundation Distance", robot.backDist.getDistance(DistanceUnit.MM));
        telemetry.update();
        //move.distance("backwards", 0.2, 0, 90, 5, robot.backDist);
        //robot.foundationServo(false);
        sleep(100);

        sleep(50000);
   /*       sleep(100);
    move.arm(1, 200);
    sleep(100);
    
    move.forward(normPow, 22, 0);
    move.turn(30);
      sleep(100);
    robot.intake("in");
    move.forward(normPow, 7, 30);
    robot.intake("stop");
    move.backward(normPow, 7, 30);
    move.turn(90);
      sleep(100);
      move.distance("backwards", slowPow, 1500, 90, 178, robot.backDist);
    move.turn(180);
      sleep(100);
    move.backward(slowPow, 5, 180);
    robot.foundationServo(true);
      sleep(100);
    move.forward(slowPow, 10, 180);
    move.turn(90);
      sleep(100);
    robot.clawServo(true);
      sleep(100);
    move.arm(armPower, 500);
      robot.clawServo(false);
      sleep(100);
    move.arm(-armPower, 500);
      sleep(100);
    robot.foundationServo(false);
      sleep(100);
    move.forward(0.6, 30, 90);*/


        // move.left();
    }

    public void backTurn(float degrees) {
        float firstAngle = (float) move.getGlobalAngle();
        while (firstAngle != degrees) {
            move.turn(firstAngle);

            move.forGyro(-0.4, firstAngle, 50);
            firstAngle -= 1;
        }
        robot.stop();
    }
}
