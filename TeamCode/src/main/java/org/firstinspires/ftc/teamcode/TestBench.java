package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Testbench")
@Disabled
public class TestBench extends LinearOpMode {

HardwareBot robot = new HardwareBot();
Movement move = new Movement(robot, this);
  @Override
    public void runOpMode() {
         robot.init(hardwareMap, telemetry, true);
          robot.foundationServo(false); 
          waitForStart();
        //   move.leftRight(normPow, 30, 30,0.9);
            move.leftRight(0.65, 27, 27,1.15);// drive forwards at 50% for 20in
          robot.stop();
          
    }
}