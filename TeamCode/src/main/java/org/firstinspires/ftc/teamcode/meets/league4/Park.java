package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Park")
public class Park extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    MoveTwo move = new MoveTwo(robot, this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, true);

        waitForStart();
        move.forward(0.6, 6, 0);
        robot.stop();
    }
}
