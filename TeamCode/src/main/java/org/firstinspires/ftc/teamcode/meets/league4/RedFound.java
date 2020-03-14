package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedFoundation")
public class RedFound extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    MoveTwo move = new MoveTwo(robot, this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, true);

        waitForStart();
        move.encoderStrafe(0.4, -20, 1.0);
        move.distance("backwards", 0.2, 0, 0, 5, robot.backDist);
        robot.foundationServo(false);
        sleep(500);
        move.forward(0.5, 40, 0);
        robot.foundationServo(true);
        sleep(500);
        //  move.turn(90);
        move.encoderStrafe(0.4, 50, 5.0);
        move.forward(0.4, 20, 0);
        robot.stop();
        sleep(1000);
    }
}
