package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

// Autonomous with R1-3
@Autonomous(name = "RedStones")
@Disabled
public class RedStones extends LinearOpMode {

    /* CREATE VARIABLES */
    String position = "none";

    /* CREATE OBJECTS */
    // Robot Objects
    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);
    ObjectDetection od = new ObjectDetection(this);
    Path path = new Path(robot, move, this);

    // Other Objects
    ElapsedTime backupTimer = new ElapsedTime();

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
                                if (midH <= 215) {
                                    position = "Left";
                                    redStonesPath(position, 0, 50, 229, 75);
                                    // redPath(position, 0, 50, 229, 80);
                                } else if (215 <= midH && midH < 425) {
                                    position = "Center";
                                    redStonesPath(position, 11, 40, 229 * 2, 65);
                                    // redPath(position, 11, 42, 229 * 2, 65);
                                } else if (425 <= midH) {
                                    position = "Right";
                                    redStonesPath(position, 22, 30, 229 * 3, 55);
                                    // redPath(position, 22, 30, 229 * 3, 55);
                                }
                            } else if (backupTimer.seconds() > 1.5) {
                                position = "Center";
                                redStonesPath(position, 11, 40, 229 * 2, 65);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        od.shutdown();
    }

    public void redStonesPath(
            String skystone,
            double strafeInches,
            double firstOut,
            double distanceToSecond,
            double outtakeDist) {
        path.redStoneOne(skystone, strafeInches);
        move.newTurn(-87); // turn to -85deg
        move.forward(0.6, firstOut, -87);
        path.redStoneTwo(skystone, distanceToSecond, outtakeDist);
    }
}
