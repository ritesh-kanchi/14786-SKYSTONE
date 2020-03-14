package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@Autonomous(name="FoundationPOC")
public class FoundationPOC extends LinearOpMode{

HardwareBot robot = new HardwareBot();
Movement move = new Movement(robot,this);
Pathways path = new Pathways(robot,move,this);
 ElapsedTime ftimer = new ElapsedTime();

@Override
public void runOpMode() {
    robot.init(hardwareMap,telemetry,true);
     robot.foundationServo(true);
    waitForStart();
    while(opModeIsActive()) {
   //  move.driveForward(0.8,7,0);
 //    sleep(500);
     
        /*move.mecDrive(0.2,0,0,false);
     sleep(250);
     ftimer.reset();
     while(ftimer.seconds() < .5) {
       
     }
     
                 robot.foundationServo(false);
                 sleep(250);
                  move.driveBackward(0.3, 48, 0);
                  sleep(100);
                robot.foundationServo(true);
                  sleep(250);
                move.intake("out",0.8);
                sleep(300);
                   move.intake("stop");
                move.encoderStrafe(0.4, -20, 1.0);
                move.encoderStrafe(0.4, -20, 1.0);
                 move.driveForward(path.normalPower, 18, 0);
                 move.turn(-89);
                 move.driveBackward(path.normalPower, 15, -89);
                sleep(40000);
        move.turn(90);
        move.mecDrive(0.3, 0, 0, false);

        // questionable code
        sleep(750);
        move.turn(90);
        move.mecDrive(0.3, 0, 0, false);
        sleep(500);
        ftimer.reset();
        robot.foundationServo(false);
        // can we change this to a 250 to match the bottom
        sleep(350);
        robot.setZero();*/
        /*move.strafeLeft(0.6, 44, 0); // follow a ratio of 1.1 and multiply wanted distance by 1.1
        move.turn(0);
        sleep(30000);*/
        /*move.strafeLeft(0.6, 66, 0);
        move.turn(0);
        sleep(1000*/
        move.strafeRight(0.6, 30, 0); // follow a ratio of TBD and multiply wanted distance by TBD
       // move.driveForward(0.4,10,0);
        sleep(30000);
         /*path.firstSkystoneRed("Left", -5, path.acrossFieldRight + 8); // added 7
                    // Outtake's the first Skystone and goes to get second
                    //skystone45Red(79);
                    skystone90Red(89);
                    path.secondSkystoneRed((path.acrossFieldRight * 1.45) + 35);*/
                    //skystoneFoundationRed();
    }
}

  public void pivTurn(long time) {
  robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      robot.leftFront.setPower(-0.6);
      robot.rightFront.setPower(-0.6);
  robot.leftBack.setPower(0);
      robot.rightBack.setPower(-0.6);
  sleep(time);
  robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }
  
   public void skystone90Red(double backDist) {
        move.turn(0);

        sleep(100);
       move.driveForward(0.6, 13, 0);
        move.intake("out", 0.3); // can we bump this up to .8
        sleep(500); // can we reduce this to 300
       move.driveBackward(0.6, 17, 0);
        move.intake("stop");
        //move.turn(-89); // if we are not strafing
        move.strafeLeft(0.6, 79, 0);
        robot.setZero();
        sleep(250);
        //move.driveBackward(path.normalPower, backDist, -89); // if we are not strafing
  }
}