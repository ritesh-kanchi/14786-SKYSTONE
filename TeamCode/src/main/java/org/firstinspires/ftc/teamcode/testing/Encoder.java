package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class Encoder extends LinearOpMode
{
  HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        // reset encoder count kept by left motor.
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
         robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.

        robot.leftFront.setTargetPosition(560-62);
        robot.rightFront.setTargetPosition(560-62);
        robot.leftBack.setTargetPosition(560-62);
        robot.rightBack.setTargetPosition(560-62);
        
        // set both motors to 25% power. Movement will start.

        robot.leftFront.setPower(0.1);
        robot.rightFront.setPower(0.1);
        robot.leftBack.setPower(0.1);
        robot.rightBack.setPower(0.1);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && robot.leftFront.isBusy() && robot.rightFront.isBusy())   
        {
            telemetry.addData("encoder-fwd", robot.leftFront.getCurrentPosition() + "  busy=" + robot.leftFront.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        robot.leftFront.setPower(0.0);
        robot.rightFront.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.rightBack.setPower(0.0);

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 1)
        {
            telemetry.addData("encoder-fwd-end", robot.leftFront.getCurrentPosition() + "  busy=" + robot.leftFront.isBusy());
            telemetry.update();
            idle();
        }

        // Now back up to starting point. In this example instead of
        // having the motor monitor the encoder, we will monitor the encoder ourselves.

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setPower(-0.25);
        robot.rightFront.setPower(-0.25);
        robot.leftBack.setPower(-0.25);
        robot.rightBack.setPower(-0.25);

        while (opModeIsActive() && robot.leftFront.getCurrentPosition() > 0)
        {
            telemetry.addData("encoder-back", robot.leftFront.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        robot.leftFront.setPower(0.0);
        robot.rightFront.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.rightBack.setPower(0.0);

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-end", robot.leftFront.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
