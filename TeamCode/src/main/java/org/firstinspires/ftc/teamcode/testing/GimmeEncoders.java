package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@TeleOp(name = "GimmeEncoders")
public class GimmeEncoders extends LinearOpMode {

  HardwareBot robot = new HardwareBot();

  @Override
  public void runOpMode() {

      robot.init(hardwareMap, telemetry, true);
     robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.setEncoders(true);
    waitForStart();
    while (opModeIsActive()) {
    
      telemetry.addData("LF given", robot.leftFront.getCurrentPosition());
      
     
      telemetry.addData("RF given", robot.rightFront.getCurrentPosition());
      
      
      telemetry.addData("LB given", robot.leftBack.getCurrentPosition());
      
     
      telemetry.addData("RB given", robot.rightBack.getCurrentPosition());
      telemetry.update();
    }
  }

  public void gap() {
    sleep(500);
  }
}
