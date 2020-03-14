package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "Drawbridge")
@Disabled
public class Drawbridge extends LinearOpMode {

  HardwareBot robot = new HardwareBot();

  @Override
  public void runOpMode() {

    robot.init(hardwareMap, telemetry);
    waitForStart();

    while (opModeIsActive()) {
      robot.drawbridge(1, 3);
    }
  }
}
