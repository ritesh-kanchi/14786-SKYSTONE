package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PushbotEncoders", group = "Pushbot")
// @Disabled
public class PushbotEncoders extends LinearOpMode {

  static final double DRIVE_SPEED = 0.5;
  /* Declare OpMode members. */
  HardwareRobot robot = new HardwareRobot(); // Use a Pushbot's hardware
  private ElapsedTime runtime = new ElapsedTime();
  
  private double lastZ;
  private int turns;

  private ElapsedTime headingTimer = new ElapsedTime();


  @Override
  public void runOpMode() {

    /*
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    robot.init(hardwareMap);
    robot.initGyro(hardwareMap);

    robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders");
    telemetry.update();

    robot.initEncoders();

    // Send telemetry message to indicate successful Encoder reset
    telemetry.addData(
        "Path0",
        "Starting at %7d :%7d",
        robot.leftFront.getCurrentPosition(),
        robot.rightFront.getCurrentPosition());
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // encoderDrive(0.25,  -14, 5.0);
    encoderStrafe(0.4, -4, 5.0); 
    encoderStrafe(0.4, -4, 5.0);
     encoderStrafe(0.4, -4, 5.0);
      encoderStrafe(0.4, -4, 5.0);
      setHeading();// S1: Forward 47 Inches with 5 Sec timeout
    // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    //  encoderDrive(DRIVE_SPEED, -12, 1.0);  // S3: Reverse 24 Inches with 4 Sec timeout

    // TESTING RIGHT STRAFE for 5 inches
    // encoderMec(DRIVE_SPEED,5,-5,-5,5,5.0);
    // encoderStrafe(DRIVE_SPEED, 16, -16, 1.0);
    // encoderStrafe(DRIVE_SPEED, -16, 16, 1.0);

    telemetry.addData("Path", "Complete");
    telemetry.update();
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
  /*
   *  Method to perfmorm a relative move, based on encoder counts.
   *  Encoders are not reset as the move is based on the current position.
   *  Move will stop if any of three conditions occur:
   *  1) Move gets to the desired position
   *  2) Move runs out of time
   *  3) Driver stops the opmode running.
   */
  public void encoderDrive(double speed, double inches, double timeoutS) {

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      int newLeftFrontTarget =
          robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
          robot.rightFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newLeftBackTarget =
          robot.leftBack.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightBackTarget =
          robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are
      // running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor
      // hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot
      // will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot
      // continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive()
          && (runtime.seconds() < timeoutS)
          && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
            "Path2",
            "Running at %7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // Stop all motion;
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // Turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // sleep(250);   // optional pause after each move
    }
  }

  public void encoderStrafe(double speed, double inches, double timeoutS) {

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      int newLeftFrontTarget =
          robot.leftFront.getCurrentPosition() + (int) (inches * robot.inchCountS);
      int newRightFrontTarget =
          robot.rightFront.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newLeftBackTarget =
          robot.leftBack.getCurrentPosition() + (int) (-inches * robot.inchCountS);
      int newRightBackTarget =
          robot.rightBack.getCurrentPosition() + (int) (inches * robot.inchCountS);

      robot.leftFront.setTargetPosition(newLeftFrontTarget);
      robot.rightFront.setTargetPosition(newRightFrontTarget);
      robot.leftBack.setTargetPosition(newLeftBackTarget);
      robot.rightBack.setTargetPosition(newRightBackTarget);

      // Turn On RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      robot.leftFront.setPower(Math.abs(speed));
      robot.rightFront.setPower(Math.abs(speed));
      robot.leftBack.setPower(Math.abs(speed));
      robot.rightBack.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are
      // running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor
      // hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot
      // will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot
      // continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive()
          && (runtime.seconds() < timeoutS)
          && (robot.leftFront.isBusy()
              && robot.rightFront.isBusy()
              && robot.leftBack.isBusy()
              && robot.rightBack.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
        telemetry.addData(
            "Path2",
            "Running at %7d :%7d",
            robot.leftFront.getCurrentPosition(),
            robot.rightFront.getCurrentPosition());
        telemetry.update();
      }

      // Stop all motion;
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);

      // Turn off RUN_TO_POSITION
      robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // sleep(250);   // optional pause after each move
    }
  }
}
