package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverControl")
public class DriverControl extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);

  @Override
  public void runOpMode() {
    robot.init(hardwareMap, telemetry);
    robot.foundationServo(false);
    waitForStart();
    while (opModeIsActive()) {
      int buttonHitX = 0;
      int buttonHitY = 0;
        // Joysticks
      double leftStickY = -gamepad1.left_stick_y;
      double leftStickX = gamepad1.left_stick_x;
      double rightStickX = gamepad1.right_stick_x;

        // Triggers
      double leftTrigger = Range.clip(+gamepad1.left_trigger, -0.35, 0.35);
      double rightTrigger = gamepad1.right_trigger;

        // Drive
      move.mecDrive(leftStickY, rightStickX, leftStickX);

        // Intake controls
      robot.leftIntake.setPower(leftTrigger - rightTrigger);
      robot.rightIntake.setPower(leftTrigger - rightTrigger);

        //Buttons
      while (gamepad1.a) {
        move.strafe("left", 1);
      }

      while (gamepad1.b) {
        move.strafe("right", 1);
      }

   /*   if (gamepad1.y) {
          robot.foundationServo(false);
      } */


      /*  if (gamepad1.x) {
          if(buttonHitX == 0 || 0 < buttonHitX) {
            telemetry.addData("X HIT",buttonHitX);
            telemetry.update();
            robot.foundationServo(true);
            sleep(500);
            buttonHitX++;
          } 
          if (buttonHitX > 0 && gamepad1.x) {
            telemetry.addData("X HIT",buttonHitX);
            telemetry.update();
            robot.foundationServo(false);
            sleep(500);
            buttonHitX = 0;
            
          }
          
      } */

        // Bumpers
        while (gamepad1.left_bumper) {
            move.mecDrive(0, -0.2, 0);
      }

        while (gamepad1.right_bumper) {
            move.mecDrive(0, 0.2, 0);
        }
        
        while(gamepad1.dpad_down){
          robot.drawbridge(1,.01);
        }
        
           while(gamepad1.dpad_up){
          robot.drawbridge(-1,.01);
        }
        
        if(gamepad1.dpad_left){
          robot.foundationServo(true);
        } 
        
        if (gamepad1.dpad_right) {
           robot.foundationServo(false);
        }
        
      
      if (gamepad1.y) {
          if(buttonHitY == 0 || 0 < buttonHitY) {
            telemetry.addData("Y HIT",buttonHitY);
            telemetry.update();
            robot.capstone(true);
            sleep(500);
            buttonHitY++;
          } 
          if (buttonHitY > 0 && gamepad1.y) {
            telemetry.addData("Y HIT",buttonHitY);
            telemetry.update();
            robot.capstone(false);
            sleep(500);
            buttonHitY = 0;
            
          }
          
      }

    }
  }
}
