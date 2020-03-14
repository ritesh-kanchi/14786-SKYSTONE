package org.firstinspires.ftc.teamcode.league1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GetServo")
public class GetServo extends LinearOpMode {

  Servo leftServo, rightServo;

  @Override
  public void runOpMode() {
    leftServo = hardwareMap.get(Servo.class, "left_servo");
    rightServo = hardwareMap.get(Servo.class, "right_servo");

    leftServo.setDirection(Servo.Direction.FORWARD);
    rightServo.setDirection(Servo.Direction.REVERSE);

    while (opModeIsActive()) {
      leftServo.setPosition(.5);
      rightServo.setPosition(.5);
      telemetry.addData("Left Servo", leftServo.getPosition());
      telemetry.addData("Right Servo", rightServo.getPosition());

      while (gamepad1.a) {
        leftServo.setPosition(.99);
        rightServo.setPosition(.99);
      }
    }
  }
}
