package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PartTwo")
@Disabled
public class PartTwo extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot,this);
    
    double normPow = 0.6;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap,telemetry,true);

        waitForStart();
        robot.clawServo(true);
        move.newTurn(90);
        sleep(250);
        move.distance1("backwards", 0.7, 1250, 90, 381, robot.backDist);
        move.newTurn(180);
        move.distance("backwards", 0.2, 0, 180, 5, robot.backDist);
        robot.foundationServo(false);
        sleep(500); // decrease?
        move.leftRight(0.5, 20,20,1); // increase?
        sleep(100);
        move.leftRight(1, 55, -55,1.5);
        robot.foundationServo(true);
        move.forwardT(-0.8,375);
        sleep(125);
        move.forwardT(0.6,275);
        move.newTurn(-85);
        sleep(100);
        move.distance1("backwards", 0.7, 1250, -85, 229, robot.backDist); // increase
        move.newTurn(35);
        robot.intake("in");
        move.forward(normPow, 8, 35); // increase
        robot.intake("stop");
        move.backward(normPow, 8, 35);// increase
        move.newTurn(-85);
        move.forward(normPow, 75, -85);// increase
        sleep(100);
        robot.intake("out", 0.25);
        sleep(650);
        robot.intake("stop");
        move.backward(normPow, 20, -85); // increase
        robot.stop();
        sleep(500000);
    }


}