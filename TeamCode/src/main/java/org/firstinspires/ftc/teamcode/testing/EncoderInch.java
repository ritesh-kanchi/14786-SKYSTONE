package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderInch")
//@Disabled
public class EncoderInch extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    
    // tick count for a REV 20:1 HD HEX to go 10 inches 
    static final int MOTOR_TICKS_COUNT = 255;


    @Override
    public void runOpMode() {
        // initialize hardwareMap from HardwareRobot
        robot.init(hardwareMap);
        
        // end of initialization
        telemetry.addData("Status:","Initialized");
        telemetry.update();
        waitForStart();
        
        // move forward for 10 inches at 50% speed
        encoderMove(1, 0.5);
        //encoderMove(-5, 0.2);
        
        // test for strafing 5 inches at 20% speed
       // encoderMec(5, 0, 0, 0.3);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderMove(double tenInches, double power) {
        // reset the encoders        
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // how many turns do I need the wheels to go?
        // the distance you need to drive with one rotation of the wheel is the circumference of the wheel
        
        // encoders only take integer tick counts
        int encoderDrivingTarget = (int)(MOTOR_TICKS_COUNT);

        robot.leftFront.setTargetPosition(encoderDrivingTarget);
        robot.rightFront.setTargetPosition(encoderDrivingTarget);
        robot.leftBack.setTargetPosition(encoderDrivingTarget);
        robot.rightBack.setTargetPosition(encoderDrivingTarget);

        // give motors speed to go that distance
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // allow drivers to see which part of autonomous is occuring at the moment
        String message = "Driving " + tenInches + " inches";

        while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {
            // essentially do nothing while you wait for the robot to finish driving to the position
            telemetry.addData("Path", message);
            telemetry.addData("Tick Count", robot.leftFront.getCurrentPosition());
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
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        sleep(250);

    }


// TESTING FUNCTION

    public void encoderMec(double inches, double forward, double turn, double strafe) {
        
        // reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // how many turns do I need the wheels to go 18 inches?
        // the distance you need to drive with one rotation of the wheel is the circumference of the wheel
       
        int encoderDrivingTarget = (int)(MOTOR_TICKS_COUNT);

        robot.leftFront.setTargetPosition(encoderDrivingTarget);
        robot.rightFront.setTargetPosition(encoderDrivingTarget);
        robot.leftBack.setTargetPosition(encoderDrivingTarget);
        robot.rightBack.setTargetPosition(encoderDrivingTarget);

        // give motors speed to go that distance
        robot.leftFront.setPower(forward + turn + strafe);
        robot.rightFront.setPower(forward - turn - strafe);
        robot.leftBack.setPower(forward + turn - strafe);
        robot.rightBack.setPower(forward - turn + strafe);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        String message = "Driving " + inches + " inches";

        while (robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) {
            // essentially do nothing while you wait for the robot to finish driving to the position
            telemetry.addData("Path", message);
            telemetry.update();
        }

        // stop all motion now that we have left the loop and the motors have found their targets
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        sleep(500);
    }
}
