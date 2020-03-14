package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareBot;
import org.firstinspires.ftc.teamcode.meets.league4.Movement;

public class FullPath {

    public static int blueValue = 450;
    public static int redValue = 450;
    public static boolean passedTape = false;

    /* OBJECTS */
    HardwareBot robot = null;
    Movement move = null;
    LinearOpMode opmode = null;

    /* CONSTRUCTOR */
    public FullPath(HardwareBot arobot, Movement amove, LinearOpMode aopmode) {
        robot = arobot;
        move = amove;
        opmode = aopmode;
    }

    public void driveDistance(int inches, double forward, double turn, double strafe, DistanceSensor distanceSensor) {
        while (distanceSensor.getDistance(DistanceUnit.INCH) > inches) {
            move.mecDrive(forward, turn, strafe, false);
        }
    }

    public void driveColor(String color, double forward, double turn, double strafe, ColorSensor colorSensor) {
        passedTape = false;
        if (color.equals("blue") || color.equals("Blue") || color.equals("BLUE")) {
            while (colorSensor.blue() < blueValue) {
                move.mecDrive(forward, turn, strafe, false);
            }
            passedTape = true;
        } else if (color.equals("red") || color.equals("Red") || color.equals("RED")) {
            while (colorSensor.red() < redValue) {
                move.mecDrive(forward, turn, strafe, false);
            }
            passedTape = true;
        }
    }

    public void getData(Recognition skystone) {
        // Math calculations and easier variable calls
        double skystoneWidth = skystone.getWidth(); // Grabs the width of th
        double skystoneHeight = skystone.getHeight(); // Grabs the height of the skystone
        double skystoneCenter =
                skystoneWidth / 2; // Grabs the center of the skystone's x coordinate, can be found by
        // dividing getLeft+getRight by 2
        double skystoneConfidence =
                skystone.getConfidence(); // Grabs how confident that what the system sees is a
        // skystone
        double frameCenterX =
                skystone.getImageWidth() / 2; // Grabs the center of the screen's x coordinate
        double frameCenterY =
                skystone.getImageHeight() / 2; // Grabs the center of the screen's y coordinate

        // Returning the data through telemetry
        opmode.telemetry.addData("Skystone", skystone); // Returns the skystone object's data
        opmode.telemetry.addData(
                "Skystone Left", skystone.getLeft()); // Returns the skystone left coordinate
        opmode.telemetry.addData("Skystone Center", skystoneCenter); // Returns skystoneCenter
        opmode.telemetry.addData("Skystone Width", skystoneWidth); // Returns skystoneWidth
        opmode.telemetry.addData("Skystone Height", skystoneHeight); // Returns skystoneHeight
        opmode.telemetry.addData(
                "Skystone Confidence", skystoneConfidence); // Returns skystoneConfidence
        opmode.telemetry.addData("Center of frame - X", frameCenterX); // Returns frameCenterX
        opmode.telemetry.addData("Center of frame - Y", frameCenterY); // Returns frameCenterY
    }
}
