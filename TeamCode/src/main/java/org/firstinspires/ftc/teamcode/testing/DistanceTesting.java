package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareBot;
import org.firstinspires.ftc.teamcode.MoveTwo;

@Autonomous(name = "Sensor: REV2mDistance", group = "Sensor")
public class DistanceTesting extends LinearOpMode {

  HardwareBot robot = new HardwareBot();
  MoveTwo move = new MoveTwo(robot, this);
  Movement moveOld = new Movement(robot, this);
  ElapsedTime moveTimer = new ElapsedTime();
  private DistanceSensor sensorRange;

  @Override
  public void runOpMode() {
    // you can use this as a regular DistanceSensor.
    sensorRange = hardwareMap.get(DistanceSensor.class, "dist_sense");

    robot.init(hardwareMap, telemetry, true);

    // you can also cast this to a Rev2mDistanceSensor if you want to use added
    // methods associated with the Rev2mDistanceSensor class.
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

    telemetry.addData(">>", "Press start to continue");
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {


      // generic DistanceSensor methods.
      telemetry.addData("deviceName", sensorRange.getDeviceName());
      telemetry.addData(
              "range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
      telemetry.addData(
              "range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
      telemetry.addData(
              "range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
      telemetry.addData(
              "range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

      // Rev2mDistanceSensor specific methods.
      telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
      telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));


      telemetry.update();
      robot.stop();
      telemetry.update();
    }
  }
}
