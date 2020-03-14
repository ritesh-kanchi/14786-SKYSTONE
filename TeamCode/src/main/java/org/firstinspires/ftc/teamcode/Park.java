package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Park")
public class Park extends LinearOpMode {

HardwareBot robot = new HardwareBot();
Movement move = new Movement(robot,this);
@Override
    public void runOpMode() {
         robot.init(hardwareMap, telemetry, true);
          robot.foundationServo(false); 
          waitForStart();
            move.forward(0.65, 10,0);
          robot.stop();
          
    }
}