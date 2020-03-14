package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="TestingEncoders")
public class TestingEncoders extends LinearOpMode {
    
    HardwareRobot robot = new HardwareRobot();
    
    @Override
    public void runOpMode() {
    // init robot hardware mapping
    robot.init(hardwareMap);
 
    telemetry.update();

    robot.initEncoders();
    
      waitForStart();
        if (opModeIsActive()) {
      while (opModeIsActive()) {
  double speed = -gamepad1.left_stick_y;
              robot.leftFront.setPower(speed);
              robot.rightFront.setPower(speed);
              robot.leftBack.setPower(speed);
              robot.rightBack.setPower(speed);
       
            telemetry.addData(
            "Path2",
            "Running at %7d :%7d :%7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition(),
            robot.leftBack.getCurrentPosition(),
            robot.rightBack.getCurrentPosition());
        telemetry.update();
      }
        }
}
}
