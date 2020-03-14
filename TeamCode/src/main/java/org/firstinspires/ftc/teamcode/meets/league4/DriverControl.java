package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverControl")
public class DriverControl extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot, this);

    double clipPower = 0.6;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false);
        robot.foundationServo(true);
        waitForStart();
        while (opModeIsActive()) {
            /* GAMEPAD 1 */

            // Joysticks
            double leftStickY1 = -gamepad1.left_stick_y;
            double leftStickX1 = gamepad1.left_stick_x;
            double rightStickX1 = gamepad1.right_stick_x;

            move.mecDrive(leftStickY1, rightStickX1, leftStickX1, clipPower);

            // Triggers
            double leftTriggerIntake = Range.clip(gamepad1.left_trigger, -0.35, 0.35);
            double rightTriggerIntake = Range.clip(gamepad1.right_trigger, -0.5, 0.5);

            robot.leftIntake.setPower(leftTriggerIntake - rightTriggerIntake);
            robot.rightIntake.setPower(leftTriggerIntake - rightTriggerIntake);

            // Buttons
            if (gamepad1.a) {
                robot.foundationServo(true);
            }

            if (gamepad1.b) {
                robot.foundationServo(false);
            }

            while (gamepad1.right_bumper) {
                clipPower = 0.3;
            }

            while (gamepad1.left_bumper) {
                clipPower = 0.6;
            }

            while (gamepad1.x) {
                move.strafe("left", 1);
            }

            while (gamepad1.y) {
                move.strafe("right", 1);
            }

            /* GAMEPAD 2 */

            // Joysticks
            double leftStickY2 = Range.clip(-gamepad2.left_stick_y, -1, 1);
            robot.drawerMotor.setPower(leftStickY2);

            // Triggers
            double leftTriggerChain = Range.clip(gamepad2.left_trigger, -0.5, 0.5);
            double rightTriggerChain = Range.clip(gamepad2.right_trigger, -0.5, 0.5);

            robot.chainMotor.setPower(rightTriggerChain - leftTriggerChain);

            // Buttons
            if (gamepad2.a) {
                robot.clawServo(true);
            }

            if (gamepad2.b) {
                robot.clawServo(false);
            }

            if (gamepad2.x) {
                robot.capstone(true);
            }

            if (gamepad2.y) {
                robot.capstone(false);
            }
        }
    }
}
