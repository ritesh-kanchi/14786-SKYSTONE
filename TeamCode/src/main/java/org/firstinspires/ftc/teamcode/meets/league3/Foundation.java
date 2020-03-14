package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Foundation")
public class Foundation extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    Movement move = new Movement(robot,this);
    Pathways path = new Pathways(robot,move,this);

    ElapsedTime gyroTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap,telemetry,true);
        waitForStart();
        while(opModeIsActive()) {
           normFound();
        }

    }
    
    public void normFound() {
          move.encoderStrafe(path.strafePower,20,1.0);
            sleep(50);
             move.encoderStrafe(path.strafePower,4,1.0);
            path.gap();
            robot.intakeDrop(true);
            path.gap();
            move.gyroTime(0.3,0,2100,gyroTimer);
            path.gap();
            robot.foundationServo(false);
            path.gap();
            robot.setZero();
            path.gap();
            move.gyroTime(-0.3,0,3650,gyroTimer);
            path.gap();
            robot.foundationServo(true);
            sleep(1000);
           gyroStrafe(0.5,0,2250,gyroTimer);
           move.turn(0);
           move.driveBackward(0.4,5,0);
            sleep(40000);
    }
    
     public void pivFound() {
           gyroStrafe(-0.5,0,950,gyroTimer);
            path.gap();
            robot.intakeDrop(true);
            path.gap();
            move.gyroTime(0.3,0,2100,gyroTimer);
            path.gap();
            robot.foundationServo(false);
            path.gap();
            robot.setZero();
            path.gap();
            move.driveBackward(0.4,10,0);
            move.turn(-90);
            path.gap();
            robot.foundationServo(true);
           move.turn(0);
           move.driveBackward(0.4,25,0);
            sleep(40000);
    }
    
    
      // used to go forwards and backwards at a certain heading and uses time
  public void gyroStrafe(double power, float wantedAngle, double msec, ElapsedTime timer) {
    double correction;
    timer.reset();
    while (timer.milliseconds() < msec) {
      correction = move.checkDirection(wantedAngle);

 robot.leftFront.setPower(-power + correction);
      robot.rightFront.setPower(power - correction);
     robot.leftBack.setPower(power - correction);
      robot.rightBack.setPower(-power + correction);
     
    }
    robot.setZero();
  }
}
