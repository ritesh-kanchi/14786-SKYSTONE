package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="Touch")
@Disabled
public class Touch extends LinearOpMode {
    
    HardwareBot robot = new HardwareBot();
    /**
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     *
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false);
        // wait for the start button to be pressed.
        
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (robot.isTouched()) {
                robot.clawServo(true);
                telemetry.addData("Touch1", "Is Pressed");
            } else {
                robot.clawServo(false);
                telemetry.addData("Touch1", "Is NOT Pressed");
            }

            telemetry.update();
        }
    }
}
