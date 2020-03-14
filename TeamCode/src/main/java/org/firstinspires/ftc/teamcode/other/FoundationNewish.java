package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Foundation")
@Disabled
public class FoundationNewish extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot,this);
    @Override
    public void runOpMode() {
        
        robot.init(hardwareMap,telemetry,true);
        
        waitForStart();
        move.distance1("backwards", 0.7, 1500, 0, 508, robot.backDist);

        /*move.distance("backwards", 0.2, 0, 0, 5, robot.backDist);

        robot.foundationServo(false);
        sleep(500);
        move.leftRight(0.5, 20,20,1);
        sleep(200);
        move.leftRight(1, 55, -55,3);
          robot.foundationServo(true);
        move.forwardT(-0.7,550);
      
        sleep(5000); */
        
        
        
    }
}