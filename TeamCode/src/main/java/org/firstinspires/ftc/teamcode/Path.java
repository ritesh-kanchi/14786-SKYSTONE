package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Path {

    HardwareBot robot = null;
    Movement move = null;
    LinearOpMode opmode = null;

    /* CREATE VARIABLES */
    // Power Variables
    double normPow = 0.65;
    double distPow = 0.75;
    double strafePow = 0.7;
    double armPow = 0.7;
    double outtakePow = 0.3;
    double intakeDist = 10;
    float stoneAngle = 35;
    long gapTime = 150;
    long outtakeTime = 600;

    // Stone Variables
    double redLeftBound = 225;
    double redRightBound = 400;
    double blueLeftBound = 215;
    double blueRightBound = 425;

    public Path(HardwareBot arobot, Movement amove, LinearOpMode aopmode) {
        robot = arobot;
        move = amove;
        opmode = aopmode;
    }

    public void redStoneOne(String skystone, double strafeInches) {
        opmode.telemetry.addData("Skystone Position", skystone); // sends skystone position to telemetry
        opmode.telemetry.update(); // update the telemetry record
        if (skystone.equals("Right")) { // specialized entry for right skystone
            move.arm(armPow, 200); // raise the arm at armPow for 175ms
            opmode.sleep(gapTime); // gapTime gap
            move.right(strafePow, strafeInches, 0, 1.2); // strafe to strafeInches at 70%
            opmode.sleep(gapTime); // gapTime gap
        } else { // nonspecialized entry for anything but skystone

            opmode.telemetry.addData("skystone", skystone);
            opmode.telemetry.update();
            // sleep(10000);
            move.right(strafePow, strafeInches, 0, 1); // strafe to strafeInches at 70%
            opmode.sleep(gapTime); // gapTime gap
            move.arm(armPow, 200); // raise the arm at armPow for 175ms
            opmode.sleep(gapTime); // gapTime gap
        }
        move.forward(normPow, 22, 0); // drive forward 22in at normPow
        move.newTurn(stoneAngle); // turn to 35deg (left)
        opmode.sleep(gapTime); // gapTime gap
        robot.intake("in"); // turn on the "in"
        move.forward(normPow, intakeDist, stoneAngle); // drive forward 7in at normPow at 35deg
        robot.intake("stop"); // turn on the "stop"
        move.backward(normPow, intakeDist, stoneAngle); // drive backward 8in at normPow at 35deg
        opmode.sleep(gapTime); // gapTime gap
    }

    public void redStoneTwo(String skystone, double distanceToSecond, double outtakeDist) {
        robot.intake("out", 0.3); // turn on the "out"
        opmode.sleep(outtakeTime); // 650ms gap
        robot.intake("stop"); // turn on the "stop"
        opmode.sleep(gapTime); // gapTime gap
        if (skystone.equals("Right")) {
            move.distance1(
                    "backwards",
                    distPow,
                    1000,
                    -85,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        } else if (skystone.equals("Center")) {
            move.distance1(
                    "backwards",
                    distPow,
                    1000,
                    -85,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        } else {
            move.distance1(
                    "backwards",
                    distPow,
                    1250,
                    -87,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        }
        // until distanceToSecond
        move.newTurn(stoneAngle); // turn to 35deg
        robot.intake("in"); // turn on the "in"
        if (skystone.equals("Right")) {
            move.forward(normPow, intakeDist + 3, stoneAngle-1);
            robot.intake("stop"); // turn on the "stop"
            move.backward(normPow, intakeDist + 3, stoneAngle);
        } else {
            move.forward(normPow, intakeDist, stoneAngle);
            robot.intake("stop"); // turn on the "stop"
            move.backward(normPow, intakeDist, stoneAngle);
        } // drive forwards at normPow for 8in at 35deg
        // drive backwards at normPow for 8in at 35deg
        move.newTurn(-84); // turn to -85deg
        move.forward(normPow+.1, outtakeDist, -84); // drive forwards at normPow for outtakeDist at -85deg
        opmode.sleep(gapTime); // gapTime gap
        robot.intake("out", 0.4); // turn on the "out"
        opmode.sleep(outtakeTime); // 650ms gap
        robot.intake("stop"); // turn on the "stop"
        move.backward(normPow, 20, -84); // drive backward at normPow for 20in at -85deg
        robot.stop(); // set all motor power to 0, stopping robot
        opmode.sleep(500000); // push into sleep so crashes don't occur
    }

    public void blueStoneOne(String skystone, double strafeInches) {
         opmode.telemetry.addData("Skystone Position", skystone); // sends skystone position to telemetry
        opmode.telemetry.update(); // update the telemetry record
        if (skystone.equals("Right")) { // specialized entry for right skystone
            move.arm(armPow, 200); // raise the arm at armPow for 175ms
            opmode.sleep(gapTime); // gapTime gap
            move.right(strafePow, strafeInches, 0, 1.2); // strafe to strafeInches at 70%
            opmode.sleep(gapTime); // gapTime gap
        } else { // nonspecialized entry for anything but skystone

            opmode.telemetry.addData("skystone", skystone);
            opmode.telemetry.update();
            // sleep(10000);
            move.right(strafePow, strafeInches, 0, 1); // strafe to strafeInches at 70%
            opmode.sleep(gapTime); // gapTime gap
            move.arm(armPow, 200); // raise the arm at armPow for 175ms
            opmode.sleep(gapTime); // gapTime gap
        }
        robot.clawServo(true); // close the claw within the robot
        if(skystone.equals("Left")) {
        move.forward(normPow, 22, 0); // drive forward 22in at normPow
        move.newTurn(stoneAngle-5); // turn to 35deg (left)
        opmode.sleep(gapTime); // gapTime gap
        robot.intake("in"); // turn on the "in"
        move.forward(normPow+4, intakeDist, stoneAngle-5); // drive forward 7in at normPow at 35deg
        robot.intake("stop"); // turn on the "stop"
        move.backward(normPow+4, intakeDist, stoneAngle-5); // drive backward 8in at normPow at 35deg
        } else {
            move.forward(normPow, 22, 0); // drive forward 22in at normPow
        move.newTurn(stoneAngle-5); // turn to 35deg (left)
        opmode.sleep(gapTime); // gapTime gap
        robot.intake("in"); // turn on the "in"
        move.forward(normPow, intakeDist, stoneAngle-5); // drive forward 7in at normPow at 35deg
        robot.intake("stop"); // turn on the "stop"
        move.backward(normPow, intakeDist, stoneAngle-5); 
        }
        opmode.sleep(gapTime); // gapTime gap
    }

    public void blueStoneTwo(String skystone, double distanceToSecond, double outtakeDist) {
        robot.intake("out", outtakePow); // turn on the "out"
        opmode.sleep(outtakeTime); // 650ms gap
        robot.intake("stop"); // turn on the "stop"
        opmode.sleep(gapTime); // gapTime gap
        if (skystone.equals("Right")) {
            move.distance1(
                    "backwards",
                    distPow,
                    1250,
                    90,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        } else if (skystone.equals("Center")) {
            move.distance1(
                    "backwards",
                    distPow,
                    1000,
                    90,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        } else {
            move.distance1(
                    "backwards",
                    distPow,
                    1000,
                    90,
                    distanceToSecond,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // -85deg
        }
        // until distanceToSecond
        move.newTurn(-stoneAngle); // turn to 35deg
        robot.intake("in"); // turn on the "in"
        if (skystone.equals("Right") || skystone.equals("Left")) {
            move.forward(normPow, intakeDist + 6, -stoneAngle);
            robot.intake("stop"); // turn on the "stop"
            move.backward(normPow, intakeDist + 6, -stoneAngle);
        } else {
            move.forward(normPow, intakeDist, -stoneAngle);
            robot.intake("stop"); // turn on the "stop"
            move.backward(normPow, intakeDist, -stoneAngle);
        } // drive forwards at normPow for 8in at 35deg
        // drive backwards at normPow for 8in at 35deg
        move.newTurn(87); // turn to -85deg
        move.forward(normPow+.1, outtakeDist, 87); // drive forwards at normPow for outtakeDist at -85deg
        opmode.sleep(gapTime); // gapTime gap
        robot.intake("out", outtakePow); // turn on the "out"
        opmode.sleep(outtakeTime); // 650ms gap
        robot.intake("stop"); // turn on the "stop"
        move.backward(normPow, 20, 87); // drive backward at normPow for 20in at -85deg
        robot.stop(); // set all motor power to 0, stopping robot
        opmode.sleep(500000); // push into sleep so crashes don't occur
    }
}
