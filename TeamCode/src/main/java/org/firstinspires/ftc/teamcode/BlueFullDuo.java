package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Autonomous with R1-3
@Autonomous(name = "BlueFullDuo")
public class BlueFullDuo extends LinearOpMode {

    /* CREATE OBJECTS */
    // Robot Objects
    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);
    ObjectDetection od = new ObjectDetection(this);
    Path path = new Path(robot, move, this);
    // Other Objects
    ElapsedTime backupTimer = new ElapsedTime();

    /* CREATE VARIABLES */
    // Power Variables
    double normPow = path.normPow;
    double distPow = path.distPow;
    long gapTime = path.gapTime;
    String position = "none";

    @Override
    public void runOpMode() {

        // Robot initialization
        robot.init(hardwareMap, telemetry, true);

        // Object Detection
        od.init();

        waitForStart();
        backupTimer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (od.tfod != null) {
                    List<Recognition> updatedRecognitions = od.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            // Data from TF+Vuforia
                            od.tfData(i, recognition);
                            i++;
                            // TF determines positon from given Vuforia data
                            if (recognition.getLabel().equals(od.LABEL_SECOND_ELEMENT)) {
                                // Skystone mid variables
                                double midH = (recognition.getLeft() + recognition.getRight()) / 2;
                                double midV = (recognition.getTop() + recognition.getBottom()) / 2;

                                // Checks mid vars to be between certain range, returns position
                                // String skystone, double strafeInches, double forDist, double mmBack,double
                                // backDist
                                if (midH <= path.blueLeftBound) {
                                    position = "Left";
                                    blueFullPath(position, 0, 240 * 3, 55);
                                } else if (path.blueLeftBound <= midH && midH < path.blueRightBound) {
                                    position = "Center";
                                    blueFullPath(position, 11, 270 * 2, 65);
                                } else if (path.blueRightBound <= midH) {
                                    position = "Right";
                                    blueFullPath(position, 20, 360, 75);
                                }
                            } else if (backupTimer.seconds() > 1.5) {
                                position = "BACKUP Center";
                                blueFullPath(position, 11, 229 * 2, 75);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        od.shutdown();
    }

    public void blueFullPath(
            String skystone, double strafeInches, double distanceToSecond, double outtakeDist) {
        path.blueStoneOne(skystone, strafeInches);
        move.newTurn(-89); // turn to 90deg, backDist facing building zone
        sleep(250); // 250ms gap
        if (skystone.equals("Right")) {
            move.distance1(
                    "backwards",
                    distPow,
                    1750,
                    -89,
                    350,
                    robot.backDist); // run distance1 function that drives backwards at 70% for 1250ms at a
            // 90deg
            // until 381mm
        } else {
            move.distance1("backwards", distPow, 1500, -89, 350, robot.backDist);
        }
        move.newTurn(-175); // turn to 180deg, backDist facing foundation
     move.distance(
                "backwards",
                0.25,
                0,
                -175,
                1,
                robot.backDist); // run distance function that drives backwards at 20% at a 180deg until 5mm
        robot.foundationServo(false); // close foundation grabbers
        sleep(500); // wait 500ms to complete foundation grab
           move.leftRight(normPow+.2, 35, 35,1.6);// drive forwards at 50% for 20in
        sleep(gapTime); // gapTime gap
        move.leftRight(
                1, -70, 70, 1.5); // make a turn at 100% power that pushes foundation horizontally
        robot.foundationServo(true); // open foundation grabbers
        move.forwardT(-0.8, 400); // drive backwards at 80% for 375ms
        sleep(125); // 125ms gap
        move.forwardT(0.6, 400); // drive forwards at 60% for 275ms
        move.newTurn(92); // turn to -85deg
        path.blueStoneTwo(skystone, distanceToSecond, outtakeDist);
    }
}
