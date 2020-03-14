package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name = "ColorSensor")
public class ColorSensor extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  Movement move = new Movement(robot,this);

  @Override
  public void runOpMode() {

      robot.init(hardwareMap, telemetry, false);
    robot.setBrake(true);
    robot.setEncoders(true);
    waitForStart();
    while (opModeIsActive()) {
    
      telemetry.addData("Red",robot.colorSensor.red());
      telemetry.addData("Green",robot.colorSensor.green());
      telemetry.addData("Blue",robot.colorSensor.blue());
      telemetry.addData("Alpha",robot.colorSensor.alpha());
      telemetry.addData("ARGB",robot.colorSensor.argb());
       telemetry.update();
    while (robot.colorSensor.blue() < 450){
 robot.leftFront.setPower(0.2);
 robot.rightFront.setPower(0.2);
 robot.leftBack.setPower(0.2);
 robot.rightBack.setPower(0.2);
   }   
      robot.setZero();
     sleep(40000);  
    }
  }

  public void gap() {
    sleep(500);
  }
}
