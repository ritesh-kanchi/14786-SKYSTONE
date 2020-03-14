package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="PartnerBackup")
//@Disabled
public class PartnerBackup extends LinearOpMode {

    // declare motors
    DcMotor leftFront, rightFront, leftBack, rightBack;

    // declare objects
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode(){
        // hardware mapping
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Ready","Start it");
        telemetry.update();
        
        waitForStart();
        timer.reset();

        while (opModeIsActive()){
            // wait 25 seconds
            while (timer.seconds() < 25) {
                telemetry.addLine("Waiting...");
                telemetry.update();
            }
            // after 25 seconds, drive straight for 3 sec
            movement(0.4,300);
        }
    }


    // function to give forward and backwards movement for a certain time
    public void movement(double forward,long milli) {
        leftFront.setPower(forward);
        rightFront.setPower(forward);
        leftBack.setPower(forward);
        rightBack.setPower(forward);
        sleep(milli);
    }
}
