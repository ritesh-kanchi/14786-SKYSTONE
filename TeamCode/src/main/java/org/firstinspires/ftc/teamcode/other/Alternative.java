package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "Alternative")
// @Disabled
public class Alternative extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);
    ElapsedTime stop = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        robot.autonInit();
        waitForStart();
        stop.reset();
        while (opModeIsActive()) {

            while (stop.seconds() < 25) {
            }

            move.driveForward(0.5, 20, 0);
            sleep(4000);
        }
    }
}
