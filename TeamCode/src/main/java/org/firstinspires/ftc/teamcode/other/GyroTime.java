package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "GyroTime")
public class GyroTime extends LinearOpMode {
    ElapsedTime gyroTimer = new ElapsedTime(); // <-- THIS TIMER IS REQUIRED
    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);

    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {
            move.gyroTime(0.2, 0, 500, gyroTimer);
        }
    }

}
