package org.firstinspires.ftc.teamcode.meets.league3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedPlus")
@Disabled
public class RedPlus extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);
  Pathways path = new Pathways(robot, move, this);
  
  ElapsedTime gyroTimer = new ElapsedTime();

  @Override
  public void runOpMode() {

    while (opModeIsActive()) {

      move.encoderStrafe(path.strafePower, -5, 1.0);
      robot.intakeDrop(true);
      // Drive Forward
      move.driveForward(path.normalPower, path.goToStone, 0);
      path.gap();
      // Turn to corner of Skystone
      move.turn(path.angleToStoneRed - 3);
      // Turn on Intake
      move.intake("in");
      sleep(400);
      // Drive forward to get Skystone
      move.driveForward(path.normalPower, path.forBack, path.angleToStoneRedF - 3);
      // Turn off Intake
      move.intake("stop");
      // Back up
      move.driveBackward(path.normalPower, path.forBack + 1, path.angleToStoneRedF - 3);
      path.gap();
      // Turn to 90ish
      move.turn(-88);
      path.gap();
      // Drive across tape
      move.driveForward(path.normalPower, path.acrossFieldRight + 1, -88);
        
      // Realign to 90ish
      move.turn(-43);
      path.gap();
      // Outtake
      move.intake("out");
      path.intakeTime();
      // Turn off Outtake
      move.intake("stop");
      path.gap();
      // Drive back to get Second Skystone
        move.turn(-88);
      move.turn(-89);
      move.driveBackward(0.5, 72, -89);

      path.gap();
      // Turn to corner of Second Skystone
      move.turn(path.angleToStoneRed - 2);
      // Turn on Intake
      move.intake("in");
      sleep(450);
      // Drive forward to get Skystone
      move.driveForward(path.normalPower, path.forBack + 5, path.angleToStoneRedF - 2);
      // Turn off Intake
      move.intake("stop");
      // Back up
      move.driveBackward(path.normalPower, path.forBack + 2, path.angleToStoneRedF - 2);
      path.gap();
      // Turn to 90ish
      move.turn(-88);
      path.gap();
      // Drive across tape
      move.driveForward(path.normalPower, (path.acrossFieldRight * 1.45), -88);

      path.gap();
      // Overloaded Outtake Function
        // Realign to 90ish
        move.turn(-43);
        path.gap();
        // Outtake
        move.intake("out");
        path.intakeTime();
        // Turn off Outtake
        move.intake("stop");
        path.gap();
        move.turn(-88);
      // Drive back onto the tape
      move.driveForward(path.slowDownPower, path.parkDist, -88);
      
      move.turn(0);
        move.gyroTime(0.3, 0, 750, gyroTimer);
        move.turn(0);
        robot.setZero();
        sleep(250);
        move.gyroTime(0.3, 0, 500, gyroTimer);
        robot.foundationServo(false);
        // can we change this to a 250 to match the bottom
        robot.setZero();
        sleep(750);
        move.gyroTime(-0.5, 0, 2000, gyroTimer);
        robot.foundationServo(true);
        sleep(250);
        move.intake("out", 0.4);
        sleep(300);
        move.intake("stop");
        move.turn(3);
        /*move.encoderStrafe(0.4, -20, 1.0);
        move.encoderStrafe(0.4, -19, 1.0);*/
        move.strafeLeft(0.4, 35, 3);
        robot.setZero();
      
      
      // Avoid code looping
      sleep(40000);
    }
  }
}
