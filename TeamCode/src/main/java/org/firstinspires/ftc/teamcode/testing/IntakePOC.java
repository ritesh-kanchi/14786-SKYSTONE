package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "IntakePOC")
public class IntakePOC extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);

  @Override
  public void runOpMode() {

    robot.init(hardwareMap, telemetry);
    robot.setBrake(true);
    robot.setEncoders(true);
    waitForStart();
    while (opModeIsActive()) {
   move.driveForward(0.4, 24,0);
     gap();
     move.turn(-40);
      intake("in");
      gap();
      move.driveForward(0.4,10,0);
      intake("stop");
      move.driveBackward(0.4,10,0);
      gap();
       move.turn(80);
      gap(); 
      move.driveForward(0.4,30,0);
      move.driveForward(0.4,30,0);
      gap();
      intake("out");
      gap();
      intake("stop");
      robot.setZero();
      sleep(4000000); 
    }
  }

  public void gap() {
    sleep(500);
  }
  
  public void intake(String option) {
    int power = 1;
    if (option.equals("in") || option.equals("In") || option.equals("IN")) {
    robot.leftIntake.setPower(-power);
    robot.rightIntake.setPower(-power);
    } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
      robot.leftIntake.setPower(power);
    robot.rightIntake.setPower(power);
    } else {
       robot.leftIntake.setPower(0);
    robot.rightIntake.setPower(0);
    }
  }
  
}
