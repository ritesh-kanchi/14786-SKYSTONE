package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Empirical")
//@Disabled
public class Empirical extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    
    // tick count for a REV 20:1 HD HEX
    static final int MOTOR_TICKS_COUNT = 560;

    
    // diameter(inches) of the GOBilda Mecanum Wheels
    static final double MOTOR_DIAMETER = 3.937;

    @Override
    public void runOpMode() {
        // initialize hardwareMap from HardwareRobot
        robot.init(hardwareMap);
        
        // end of initialization
        telemetry.addData("Status:","Initialized");
        telemetry.update();
        waitForStart();
        
    
        // reset the encoders        
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // how many turns do I need the wheels to go?
  /*      // the distance you need to drive with one rotation of the wheel is the circumference of the wheel
        int rotations = 1;
        
        // encoders only take integer tick counts
        int encoderDrivingTarget = rotations*500;

        robot.leftFront.setTargetPosition(encoderDrivingTarget);
        robot.rightFront.setTargetPosition(encoderDrivingTarget);
        robot.leftBack.setTargetPosition(encoderDrivingTarget);
        robot.rightBack.setTargetPosition(encoderDrivingTarget);

        // give motors speed to go that distance
        robot.leftFront.setPower(0.2);
        robot.rightFront.setPower(0.2);
        robot.leftBack.setPower(0.2);
        robot.rightBack.setPower(0.2);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // allow drivers to see which part of autonomous is occuring at the moment
        String message = "Driving " + rotations + " rotations";

        while (opModeIsActive() && robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) {
            // essentially do nothing while you wait for the robot to finish driving to the position
            telemetry.addData("Path", message);
            telemetry.update();
        }
        
         // stop all motion now that we have left the loop and the motors have found their targets
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        
        // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
            while (1==1) {
                
                telemetry.addData("Ticks",robot.leftFront.getCurrentPosition());
                telemetry.update();
            }

    }
}
