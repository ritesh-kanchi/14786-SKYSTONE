package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Autonomous with RFoundation
@Autonomous(name = "RedFoundation")
public class RedFoundation extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);
  Pathways path = new Pathways(robot,move,this);

  @Override
  public void runOpMode() {

    robot.init(hardwareMap, telemetry);
    robot.autonInit();
    waitForStart();
    
    while (opModeIsActive()) {
    path.getFoundation();
    path.parkFoundation();
    }
  }

}
