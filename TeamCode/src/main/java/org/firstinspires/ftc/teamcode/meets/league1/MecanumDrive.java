package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends LinearOpMode {

  HardwareRobot robot = new HardwareRobot();

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    robot.init(hardwareMap);
    robot.upLeft();
    robot.upRight();
    telemetry.addData("TeleOp","Initialized");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      double leftStickY = -gamepad1.left_stick_y;
      double leftStickX = gamepad1.left_stick_x;
      double rightStickX = gamepad1.right_stick_x;

      mecDrive(leftStickY, rightStickX, leftStickX);

      while (gamepad1.left_bumper) {
        mecDrive(0, 0.2, 0);
      }

      while (gamepad1.right_bumper) {
        mecDrive(0, -0.2, 0);
      }

      if (gamepad1.x) {
        robot.leftArm.setPosition(0.4);
        robot.rightArm.setPosition(0.45);
      }

      if (gamepad1.y) {
        robot.upLeft();
        robot.upRight();
      }
      
      while (gamepad1.a) {
          robot.leftFront.setPower(1);
          robot.rightFront.setPower(-1);
          robot.leftBack.setPower(-1);
          robot.rightBack.setPower(1);
      }
      
      while (gamepad1.b) {
          robot.leftFront.setPower(-1);
          robot.rightFront.setPower(1);
          robot.leftBack.setPower(1);
          robot.rightBack.setPower(-1);
      }
      
      robot.leftIntake.setPower(Range.clip(+gamepad1.left_trigger,-0.35,0.35) - gamepad1.right_trigger);
      robot.rightIntake.setPower(Range.clip(+gamepad1.left_trigger,-0.35,0.35) - gamepad1.right_trigger);
    }
  }

  public void mecDrive(double forward, double turn, double strafe) {
    robot.leftFront.setPower(Range.clip(forward + turn + strafe, -0.5, 0.5));
    robot.rightFront.setPower(Range.clip(forward - turn - strafe, -0.5, 0.5));
    robot.leftBack.setPower(Range.clip(forward + turn - strafe, -0.5, 0.5));
    robot.rightBack.setPower(Range.clip(forward - turn + strafe, -0.5, 0.5));
  }
}