package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "ObjectPath")
// @Disabled
public class ObjectPath extends LinearOpMode {

   // robot specific objects
  HardwareRobot robot = new HardwareRobot();
  ElapsedTime runtime = new ElapsedTime();


  private double lastZ;
  private int turns;

  private ElapsedTime headingTimer = new ElapsedTime();

  @Override
  public void runOpMode() {
    // init robot hardware mapping
    robot.init(hardwareMap);

    //  init gyro and imu
    robot.initGyro(hardwareMap);
    heading();

    // bring both servos up from current position
    robot.upLeft();
    robot.upRight();

    // give drive brake zero power behavior
    robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    telemetry.addData("Status", "Resetting Encoders");
    telemetry.update();

    // init robot encoders and calibration
    robot.initEncoders();

  
    telemetry.addData(
            "Path0",
            "Starting at %7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition());
    telemetry.update();
    telemetry.addData("Autonomous Status", "Initialized");
    telemetry.update();

    waitForStart();

    // scanning loop
                  encoderDrive(0.3, -14, 1.0);
                  setHeading();
                  encoderDrive(0.3, -14, 1.0);
                  gap();
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 15, 1.0);
                  gap();
                  encoderStrafe(.8, 130, 1.0);
                  gap();
                  robot.upLeft();
                  encoderStrafe(.8, -200, 1.0);
                  encoderStrafe(.8, -20, 1.0);
                  encoderStrafe(.3, -50, 1.0);
                  
                  gap();
                  encoderDrive(0.3,40,1.0);
                  gap();
                  encoderStrafe(.3,6, 1.0);
                  gap();
                
            
                  
                  encoderDrive(0.3, -14, 1.0);
                  setHeading();
                  encoderDrive(0.3, -15, 1.0);
                  sleep(250);
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 17, 1.0);
                  gap();
                  encoderStrafe(0.7, 170, 1.0);
                  setHeading();
                  encoderStrafe(0.7, 170, 1.0);
                  setHeading();
                  robot.upLeft();
                 
                  encoderStrafe(0.4, -30, 1.0);
                  gap();
                  encoderDrive(0.4, -5, 1.0);

           /* center
                  encoderStrafe(0.25, -5.6, 1.0);
                  gap();
                  encoderDrive(0.3, -15, 1.0);
                  encoderDrive(0.3, -15, 1.0);
                  gap();
                  robot.downLeft();
                  gap();
                  encoderDrive(0.3, 6.75, 1.0);
                  gap();
                  encoderStrafe(0.25, 14, 1.0);
                  gap();
                  encoderStrafe(0.25, 14, 1.0);
                  robot.upLeft();
                  encoderStrafe(0.25, -14, 1.0);
                  gap();
                  encoderStrafe(0.25, -14, 1.0);
                  gap();
                  encoderStrafe(0.25, -14, 1.0);
                  gap();
                  encoderStrafe(0.25, 7, 1.0);
                  gap();
                  encoderDrive(0.3, -14, 1.0);
                  gap();
                  encoderDrive(0.3, 14, 1.0);
                  robot.downLeft();
                  encoderDrive(0.3, 14, 1.0);
                  gap();
                  encoderStrafe(0.25, 14, 1.0);
                  gap();
                  encoderStrafe(0.25, 14, 1.0);
                  robot.upLeft();
                  gap();
                  encoderStrafe(0.25, -14, 1.0);
                  gap();
                  */
/* right
                  encoderStrafe(0.25, -4, 1.0);
                  gap();
                  encoderDrive(0.3, -15, 1.0);
                  encoderDrive(0.3, -15, 1.0);
                  gap();
                  robot.downRight();
                  gap();
                  encoderDrive(0.3, 7.5, 1.0);
                  gap();
                  encoderStrafe(1, 14, 1.0);
                  gap();
                  encoderStrafe(1, 14, 1.0);
                  robot.upLeft();
                  encoderStrafe(1, -14, 1.0);
                  gap();
                  encoderStrafe(1, -14, 1.0);
                  gap();
                  encoderStrafe(1, -14, 1.0);
                  gap();
                  encoderStrafe(1, 7, 1.0);
                  gap();
                  encoderDrive(0.3, -14, 1.0);
                  gap();
                  encoderDrive(0.3, 14, 1.0);
                  robot.downLeft();
                  encoderDrive(0.3, 14, 1.0);
                  gap();
                  encoderStrafe(1, 14, 1.0);
                  gap();
                  encoderStrafe(1, 14, 1.0);
                  robot.upLeft();
                  gap();
                  encoderStrafe(1, -14, 1.0);
                  gap(); */
  }
    


  public void setHeading() { //target will be the angle you want to change by,
    robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // front left motor
    robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // front right motor
    robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // back left motor
    robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // back right motor
    headingTimer.reset();
    double rw;
    while(opModeIsActive()) {
      if (headingTimer.seconds() > 0.75) break;       // give the robot .75 seconds to correct its heading
      //rw = 0.02*(-1*heading());
      rw = 0.02*(heading());
      robot.leftFront.setPower(rw); // front left motor
      robot.rightFront.setPower(-rw);  // front right motor
      robot.leftBack.setPower(rw); // back left motor
      robot.rightBack.setPower(-rw); // back right motor
    }
    robot.leftFront.setPower(0); // front left motor
    robot.rightFront.setPower(0);  // front right motor
    robot.leftBack.setPower(0); // back left motor
    robot.rightBack.setPower(0); // back right motor
  }
  // private int lastZ; private int turns; these two variables should be in the class, this function returns the heading or angle of the robot since the initialization.
  public double heading() {
    Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX,
            AngleUnit.DEGREES);
    double Z = angles.firstAngle;
    if (Z > 140 && lastZ < -140) turns--;
    if (Z < -140 && lastZ > 140) turns++;
    lastZ = Z;
    return turns * 360.0 + Z;
  }

  
  // forward/backward with encoders
  public void encoderDrive(double speed, double inches, double timeoutS) {
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // make sure opmode is still active
    if (opModeIsActive()) {

      // calculate new target position
      int newLeftFrontTarget =
              robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newRightFrontTarget =
              robot.rightFront.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newLeftBackTarget =
              robot.leftBack.getCurrentPosition() + (int) (inches * robot.inchCount);
      int newRightBackTarget =
              robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCount);

      // set new position
      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // turn on RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset time and start motion
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      while (opModeIsActive()
              && (runtime.seconds() < timeoutS)
              && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

        // show running telemetry
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
                "Path2",
                "Running at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // stop all motion
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  // strafe with encoders
  public void encoderStrafe(double speed, double inches, double timeoutS) {
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // make sure opmode is still active
    if (opModeIsActive()) {

      // calculate new target position
      int newLeftFrontTarget =
              robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
              robot.rightFront.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newLeftBackTarget =
              robot.leftBack.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newRightBackTarget =
              robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

      // set new target position
      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // turn on RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode
              .RUN_TO_POSITION);

      // reset time and start motion
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      while (opModeIsActive()
              && (runtime.seconds() < timeoutS)
              && (robot.leftFront.isBusy()
              || robot.rightFront.isBusy()
              || robot.leftBack.isBusy()
              || robot.rightBack.isBusy())) {

        // show running telemetry
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
                "Path2",
                "Running at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // stop all motion
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }
  
  // strafe with encoders
  public void encoderDiag(double speed, double inches, double timeoutS) {
  //  robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  //  robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // make sure opmode is still active
    if (opModeIsActive()) {

      // calculate new target position
     // int newLeftFrontTarget =
             // robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
              robot.rightFront.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newLeftBackTarget =
              robot.leftBack.getCurrentPosition() + (int) (-inches * robot.inchCountS);
     // int newRightBackTarget =
          //    robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

      // set new target position
   //   robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
  //    robot.rightBack.setTargetPosition(newRightBackTarget);

      // turn on RUN_TO_POSITION
   //   robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   //   robot.rightBack.setMode(DcMotor.RunMode
     //         .RUN_TO_POSITION);

      // reset time and start motion
      runtime.reset();
robot.leftFront.setPower(Math.abs(0));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
     robot.rightBack.setPower(Math.abs(0));

      while (opModeIsActive()
              && (runtime.seconds() < timeoutS)
              && (robot.rightFront.isBusy()
              || robot.leftBack.isBusy())) {

        /* show running telemetry
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
                "Path2",
                "Running at %7d :%7d",
               // robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update(); */
      }

      // stop all motion
    //  robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
   //   robot.rightBack.setPower(0);

      // turn off RUN_TO_POSITION
    //  robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   //   robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  // half-second gap function
  public void gap() {
    sleep(250);
    setHeading();
  }
}
