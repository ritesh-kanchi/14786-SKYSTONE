package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "DetectionTest")
@Disabled
public class DetectionTest extends LinearOpMode {

    String position = "none";
    /* CREATE OBJECTS */
    // Robot Objects
    HardwareBot robot = null;
    Movement move = null;
    ObjectDetection od = new ObjectDetection(this);
    Path path = new Path(robot, move, this);

    @Override
    public void runOpMode() {

        // Object Detection
        od.init();

        telemetry.addLine("Detection Ready!");
        telemetry.update();

        waitForStart();

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
                                if (midH <= path.redLeftBound) {
                                    position = "R-Left";
                                       telemetry.addData("Red Position", position);

                                } else if (path.redLeftBound <= midH && midH < path.redRightBound) {
                                    position = "R-Center";
                                       telemetry.addData("Red Position", position);

                                } else if (path.redRightBound <= midH) {
                                    position = "R-Right";
                                     telemetry.addData("Red Position", position);
                                }

                                if (midH <= path.blueLeftBound) {
                                    position = "B-Left";
                                     telemetry.addData("Blue Position", position);

                                } else if (path.blueLeftBound <= midH && midH < path.blueRightBound) {
                                    position = "B-Center";
                                     telemetry.addData("Blue Position", position);

                                } else if (path.blueRightBound <= midH) {
                                    position = "B-Right";
                                     telemetry.addData("Blue Position", position);
                                }
                            }
                           
                        }
                        telemetry.update();
                    }
                }
            }
        }

        od.shutdown();
    }
}
