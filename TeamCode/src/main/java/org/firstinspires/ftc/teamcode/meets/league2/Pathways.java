package org.firstinspires.ftc.teamcode.meets.league2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Pathways {

  /* VARIABLES */
  // Power Variables
  static final double normalPower = 0.6;
  static final double slowDownPower = 0.3;
  static final double strafePower = 0.3;

  // Distance Variables
  static final double goToStone = 22;
  static final double acrossFieldLeft = 50;
  static final double acrossFieldRight = 50;
  static final double forBack = 13;
  static final double parkDist = 15;

  // Angle Variables
  static final double angleToStoneBlue = 28;
  static final double angleToStoneRed = 28;
  static final float angleToStoneBlueF = 28;
  static final float angleToStoneRedF = 28;

  /* OBJECTS */
  HardwareBot robot = null;
  Movement move = null;
  LinearOpMode opmode = null;

  /* CONSTRUCTOR */
  public Pathways(HardwareBot arobot, Movement amove, LinearOpMode aopmode) {
    robot = arobot;
    move = amove;
    opmode = aopmode;
  }

  /* FUNCTIONS */
  // Global Functions
  public void gap() {
    opmode.sleep(150);
  }

  public void intakeTime() {
    opmode.sleep(300);
  }

  // Red Pathway Functions
  public void firstSkystoneRed(String location, double strafeInches, double forDist) {
    // Return location of the Skystone
    opmode.telemetry.addData("SkyStone", location);
    opmode.telemetry.update();
    // Lower Intake
    // Strafe to get corner of First Skystone
    move.encoderStrafe(strafePower, strafeInches, 1.0);
    robot.drawbridge(1, 3.25);
    // Drive Forward
    move.driveForward(normalPower, goToStone, 0);
    gap();
    // Turn to corner of Skystone
    move.turn(angleToStoneRed - 3);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(250);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack, angleToStoneRedF - 3);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack-3, angleToStoneRedF - 3);
    gap();
    // Turn to 90ish
    move.turn(-88);
    gap();
    // Drive across tape
    move.driveForward(normalPower, forDist, -88);
  }

  public void outtakeRed(double backDist) {
    // Realign to 90ish
    move.turn(-89);
    gap();
    // Outtake
    move.intake("out");
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
    // Drive back to get Second Skystone
    move.driveBackward(normalPower, backDist, -89);
  }

  // Overloaded for Second Skystone
  public void outtakeRed() {
    // Realign to 90ish
    move.turn(-89);
    gap();
    // Outtake
    move.intake("out");
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
  }

  public void secondSkystoneRed(double acrossDist) {
    gap();
    // Turn to corner of Second Skystone
    move.turn(angleToStoneRed - 2);
    // Turn on Intake
    move.intake("in");
    intakeTime();
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack + 5, angleToStoneRedF - 2);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack, angleToStoneRedF - 2);
    gap();
    // Turn to 90ish
    move.turn(-88);
    gap();
    // Drive across tape
    move.driveForward(normalPower, acrossDist, -88);
  }

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
  public void firstSkystoneBlue(String location, double strafeInches, double forDist) {
    // Return location of the Skystone
    opmode.telemetry.addData("SkyStone", location);
    opmode.telemetry.update();
    // Lower Intake
    // Strafe to get corner of First Skystone
    move.encoderStrafe(strafePower, strafeInches, 1.0);
    robot.drawbridge(1, 3.25);
    // Drive Forward
    move.driveForward(normalPower, goToStone, 0);
    gap();
    // Turn to corner of Skystone
    move.turn(angleToStoneRed - 3);
    // Turn on Intake
    move.intake("in");
    opmode.sleep(250);
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack, angleToStoneBlueF - 3);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack+2, angleToStoneBlueF - 3);
    gap();
    // Turn to 90ish
    move.turn(88);
    gap();
    // Drive across tape
    move.driveForward(normalPower, forDist, 88);
  }

  public void outtakeBlue(double backDist) {
    // Realign to 90ish
    move.turn(89);
    gap();
    // Outtake
    move.intake("out");
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
    // Drive back to get Second Skystone
    move.driveBackward(normalPower, backDist, 89);
  }

  // Overloaded for Second Skystone
  public void outtakeBlue() {
    // Realign to 90ish
    move.turn(89);
    gap();
    // Outtake
    move.intake("out");
    intakeTime();
    // Turn off Outtake
    move.intake("stop");
    gap();
  }

  public void secondSkystoneBlue(double acrossDist) {
    gap();
    // Turn to corner of Second Skystone
    move.turn((angleToStoneBlueF - 2)*-1);
    // Turn on Intake
    move.intake("in");
    intakeTime();
    // Drive forward to get Skystone
    move.driveForward(normalPower, forBack, (angleToStoneBlueF - 2)*-1);
    // Turn off Intake
    move.intake("stop");
    // Back up
    move.driveBackward(normalPower, forBack, (angleToStoneBlueF - 2)*-1);
    gap();
    // Turn to 90ish
    move.turn(88);
    gap();
    // Drive across tape
    move.driveForward(normalPower, acrossDist, 88);
  }

  public void parkBlue() {
    gap();
    // Overloaded Outtake Function
    outtakeBlue();
    // Drive back onto the tape
    move.driveBackward(slowDownPower, parkDist+2, 88);
    // Strafe to the side to enable larger robot to fit besides us
    move.encoderStrafe(strafePower, 6, 1.0);
    // Avoid code looping
    opmode.sleep(40000);
  }

  

    // Foundation Functions
    public void getFoundation() {
      // CODE GOES HERE
    }

    public void parkFoundation() {
        // CODE GOES HERE
    }
}