package org.firstinspires.ftc.teamcode.meets.league3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverControl")
public class DriverControl extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot, this);

  @Override
  public void runOpMode() {
    robot.init(hardwareMap, telemetry, false);
    robot.foundationServo(false);
    robot.intakeDrop(false);
    waitForStart();
    while (opModeIsActive()) {
      boolean buttonHitX = true;
      boolean buttonHitY = false;
      // Joysticks
      double leftStickY = -gamepad1.left_stick_y;
      double leftStickX = gamepad1.left_stick_x;
      double rightStickX = gamepad1.right_stick_x;

      // Triggers
      double leftTrigger = Range.clip(+gamepad1.left_trigger, -0.35, 0.35);
      double rightTrigger = gamepad1.right_trigger;

      // Drive
      move.mecDrive(leftStickY, rightStickX, leftStickX, true);

      // Intake controls
      robot.leftIntake.setPower(leftTrigger - rightTrigger);
      robot.rightIntake.setPower(leftTrigger - rightTrigger);

      // Buttons
      while (gamepad1.a) {
        move.strafe("left", 1);
      }

      while (gamepad1.b) {
        move.strafe("right", 1);
      }

      // Bumpers
      while (gamepad1.left_bumper) {
        move.mecDrive(0, -0.2, 0, true);
      }

      while (gamepad1.right_bumper) {
        move.mecDrive(0, 0.2, 0, true);
      }

      if (gamepad1.dpad_left) {
        robot.foundationServo(true);
      }

      if (gamepad1.dpad_right) {
        robot.foundationServo(false);
      }
      
      if(gamepad1.x) {
        robot.capstone(true);
      }
      
      if(gamepad1.y) {
        robot.capstone(false);
      }
      
      /* beta testing double hitting
      if (gamepad1.x) {
        if (buttonHitX) {
          robot.foundationServo(true);
          buttonHitX = false;
        } else {
          robot.foundationServo(false);
          buttonHitX = true;
        }
      } */
    }
  }
}