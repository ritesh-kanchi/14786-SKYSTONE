package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TurnTest")
@Disabled
public class TurnTest extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);
    
    
      static final double fastTurnPower = 0.4;
  static final double driveClip = 0.6;
  static final double slowTurnPower = 0.2;
  static final double degRange = 1;

    @Override
    public void runOpMode() {
        
        robot.init(hardwareMap,telemetry,true);
        robot.clawServo(true);

        waitForStart(); 
        // 90 is 90
        // -90 is 80
        // 30 is 30
        // 180 is 180
        turn(30);
         turn(90);
        turn(180);
    }
    
      public void ninetyDeg(double polarity) {
        double degreeDiff = (90*polarity) - move.getGlobalAngle();
        double turnPower;

        while (Math.abs(degreeDiff) > 1) {
            telemetry.addData("DegDiff", degreeDiff);
            telemetry.update();

            turnPower = degreeDiff * 0.0095;

            if (Math.abs(degreeDiff) > 10) {
                robot.leftFront.setPower(-turnPower);
                robot.rightFront.setPower(turnPower);
                robot.leftBack.setPower(-turnPower);
                robot.rightBack.setPower(turnPower);
            }

            degreeDiff = (90*polarity) - move.getGlobalAngle();
        }
        robot.stop();
    }
    
    
    
    // turns to a given heading
  public void turn(double degrees) {
    double degreeDiff = degrees - move.getGlobalAngle();
    // rotate until turn is completed.
    while (Math.abs(degreeDiff) > 0.5) {
      telemetry.addData("DegDiff", degreeDiff);
      telemetry.update();
      if (Math.abs(degreeDiff) > 20) { // Fast turn
        if (degreeDiff > 0) { // Left turn
          robot.leftFront.setPower(-fastTurnPower);
          robot.rightFront.setPower(fastTurnPower);
          robot.leftBack.setPower(-fastTurnPower);
          robot.rightBack.setPower(fastTurnPower);
        } else { // Right turn
          robot.leftFront.setPower(fastTurnPower);
          robot.rightFront.setPower(-fastTurnPower);
          robot.leftBack.setPower(fastTurnPower);
          robot.rightBack.setPower(-fastTurnPower);
        }
      } else { // Slow turn
        if (degreeDiff > 0) { // Left turn
          robot.leftFront.setPower(-slowTurnPower);
          robot.rightFront.setPower(slowTurnPower);
          robot.leftBack.setPower(-slowTurnPower);
          robot.rightBack.setPower(slowTurnPower);
        } else { // Right turn
          robot.leftFront.setPower(slowTurnPower);
          robot.rightFront.setPower(-slowTurnPower);
          robot.leftBack.setPower(slowTurnPower);
          robot.rightBack.setPower(-slowTurnPower);
        }
      }

      degreeDiff = degrees - move.getGlobalAngle();
    }

    robot.stop();
  }

    
}
