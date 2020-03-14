package org.firstinspires.ftc.teamcode.meets.league3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Pathways {
  
  float addedValue = 0;

  /* VARIABLES */
  // Power Variables
  static final double normalPower = 0.5;
  static final double slowDownPower = 0.3;
  static final double strafePower = 0.3;

  // Distance Variables
  static final double goToStone = 22;
  static final double acrossFieldLeft = 50;
  static final double acrossFieldRight = 50;
  static final double forBack = 13;
  static final double parkDist = 15;

  static final double rightSkystone1 = 51;

  // Angle Variables
  static final double angleToStoneBlue = 28;
  static final double angleToStoneRed = 28;
  static final float angleToStoneBlueF = 28;
  static final float angleToStoneRedF = 28;

  /* OBJECTS */
  HardwareBot robot = null;
  Movement move = null;
  LinearOpMode opmode = null;

  ElapsedTime gyroTimer = new ElapsedTime();

  /* CONSTRUCTOR */
  public Pathways(HardwareBot arobot, Movement amove, LinearOpMode aopmode) {
    robot = arobot;
    move = amove;
    opmode = aopmode;
  }

  /* FUNCTIONS */
  // Global Functions
  // small sleep used throughout for spacing purposes
  public void gap() {
    opmode.sleep(150);
  }

  // time used to run intake
  public void intakeTime() {
    opmode.sleep(300);
  }

  // Red Pathway Functions
  // gets the first skystone and drives across the tape
  public void firstSkystoneRed(String location, double strafeInches, double forDist) {
    // Return location of the Skystone
    opmode.telemetry.addData("SkyStone", location);
    opmode.telemetry.update();
    // Lower Intake
    // Strafe to get corner of First Skystone
    move.encoderStrafe(strafePower, strafeInches, 1.0);
    move.gyroTime(-0.25, 0, 250, gyroTimer);
    robot.intakeDrop(true);
    // Drive Forward
    move.driveForward(normalPower, goToStone, 0);
    gap();
    // Turn to corner of Skystone
    move.turn(angleToStoneRed - addedValue);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(400);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack-1, angleToStoneRedF - addedValue);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack + 1, angleToStoneRedF - addedValue);
    gap();
    // Turn to 90ish
    move.turn(-87);
    gap();
    // Drive across tape
    move.driveForward(normalPower, forDist, -87);
  }

  // outtakes a skystone and goes backwards across the tape
  public void outtakeRed(double backDist) {
    // Realign to 90ish
    move.turn(-87);
    gap();
    // Outtake
    move.intake("out",0.5);
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
    // Drive back to get Second Skystone
    move.turn(-89);
    move.driveBackward(0.5, backDist, -89);
  }

  // Overloaded for Second Skystone
  public void outtakeRed() {
    // Realign to 90ish
    move.turn(-89);
    gap();
    // Outtake
    move.intake("out",0.5);
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
  }

  // gets the second skystone and drives across the tape
  public void secondSkystoneRed(double acrossDist) {
    gap();
    // Turn to corner of Second Skystone
    move.turn(angleToStoneRed - addedValue);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(450);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack + 5, angleToStoneRedF - addedValue);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack + 2, angleToStoneRedF - addedValue);
    gap();
    // Turn to 90ish
    move.turn(-88);
    gap();
    // Drive across tape
    move.driveForward(normalPower, acrossDist, -88);
  }

  // outtakes the second skystone and drives under the bridge and strafes near the bridge
  public void parkRed() {
    gap();
    // Overloaded Outtake Function
    outtakeRed();
    // Drive back onto the tape
    move.driveBackward(slowDownPower, parkDist, -88);
    // Strafe to the side to enable larger robot to fit besides us
    move.encoderStrafe(strafePower, -6, 1.0);
    // Avoid code looping
    opmode.sleep(40000);
  }

  // Blue Pathway Functions
  // gets the first skystone and drives across the tape
  public void firstSkystoneBlue(String location, double strafeInches, double forDist) {
    // Return location of the Skystone
    opmode.telemetry.addData("SkyStone", location);
    opmode.telemetry.update();
    // Lower Intake
    // Strafe to get corner of First Skystone
   move.encoderStrafe(strafePower, strafeInches, 1.0);
   move.gyroTime(-0.25, 0, 250, gyroTimer);
    opmode.sleep(100);
    robot.intakeDrop(true);
    // Drive Forward
    move.driveForward(normalPower, goToStone, 0);
    gap();
    // Turn to corner of Skystone
    move.turn(angleToStoneRed - addedValue);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(250);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack-2, angleToStoneBlueF - addedValue);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack + 1, angleToStoneBlueF - addedValue);
    gap();
    // Turn to 90ish
    move.turn(91);
    gap();
    // Drive across tape
    move.driveForward(normalPower, forDist, 91);
  }

  // outtakes a skystone and goes backwards across the tape
  public void outtakeBlue(double backDist) {
    // Realign to 90ish
    move.turn(91);
    gap();
    // Outtake
    move.intake("out",0.5);
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
    // Drive back to get Second Skystone
    move.turn(89);
    move.driveBackward(normalPower, backDist, 89);
  }

  // Overloaded for Second Skystone
  public void outtakeBlue() {
    // Realign to 90ish
    move.turn(89);
    gap();
    // Outtake
    move.intake("out",0.5);
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
  }

  // gets the second skystone and drives across the tape
  public void secondSkystoneBlue(double acrossDist) {
    gap();
    // Turn to corner of Second Skystone
    move.turn((angleToStoneBlueF - addedValue) * -1);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(400);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack, (angleToStoneBlueF - addedValue) * -1);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack+2, (angleToStoneBlueF - addedValue) * -1);
    gap();
    // Turn to 90ish
    move.turn(91);
    gap();
    // Drive across 89
    move.driveForward(normalPower, acrossDist, 91);
  }

  // outtakes the second skystone and drives under the bridge and strafes near the bridge
  public void parkBlue() {
    gap();
    // Overloaded Outtake Function
    outtakeBlue();
    // Drive back onto the tape
    move.driveBackward(slowDownPower, parkDist + 2, 89);
    // Strafe to the side to enable larger robot to fit besides us
    move.encoderStrafe(strafePower, 6, 1.0);
    // Avoid code looping
    opmode.sleep(40000);
  }

}