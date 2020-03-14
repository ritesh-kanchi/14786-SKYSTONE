package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// Stores all hardware and core mechanisms
public class HardwareBot {
  /* VARIABLES */
  // Encoder Strafe Variables
  static final double countsPerRev = 537.6;
  static final double wheelDia =
          100 * 0.0393701; // converting diameter from 100mm to inches, 0.0393701: conversion factor
  static final double fudgeS = 0.867667222749535; // maybe 0.283660438206579
  static final double inchCountS =
          (countsPerRev * fudgeS) / (wheelDia * 3.1415); // (counts*fudge)/circumference

  // Hardware Variables
  // Motors
  public DcMotor leftFront, rightFront, leftBack, rightBack;
  public DcMotor leftIntake, rightIntake;
  public DcMotor chainMotor;
  public DcMotor drawerMotor;
  // Servos
  public Servo foundationLeft;
  public Servo foundationRight;
  public Servo foundationMiddle;
  public Servo capstoneServo;
  public Servo stonePush;
  public Servo clawFront, clawBack;
  // Sensors
  //public ColorSensor bottomColor;
  public DistanceSensor backDist;
  // public DistanceSensor frontDist;
    public DigitalChannel touchSensor;

  /* OBJECTS */
  // Time
  public ElapsedTime stop = new ElapsedTime();
  // Gyro/IMU
  public BNO055IMU imu;
  public Orientation lastAngles = new Orientation();
  public double globalAngle;

  // Init Objects
  HardwareMap hardwareMap = null;
  Telemetry telemetry = null;

  /* CONSTRUCTOR */
  public HardwareBot() {}

  /* FUNCTIONS */
  // Global Functions
  // contains all functions for initialization to be called from multiple files
  public void init(HardwareMap hwMap, Telemetry tm, boolean auton) {

    // Link Init Objects
    hardwareMap = hwMap;
    telemetry = tm;

    /* HARDWARE MAP */
    // Motors
    leftFront = hardwareMap.get(DcMotor.class, "left_front");
    rightFront = hardwareMap.get(DcMotor.class, "right_front");
    leftBack = hardwareMap.get(DcMotor.class, "left_back");
    rightBack = hardwareMap.get(DcMotor.class, "right_back");

    leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
    rightIntake = hardwareMap.get(DcMotor.class, "right_intake");

    chainMotor = hardwareMap.get(DcMotor.class, "chain_motor");
    drawerMotor = hardwareMap.get(DcMotor.class, "drawer_motor");

    // Servos
    foundationLeft = hardwareMap.get(Servo.class, "foundation_left");
    foundationRight = hardwareMap.get(Servo.class, "foundation_right");
    foundationMiddle = hardwareMap.get(Servo.class, "foundation_middle");
    capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");
    stonePush = hardwareMap.get(Servo.class, "stone_push");

    clawFront = hardwareMap.get(Servo.class, "claw_front");
    clawBack = hardwareMap.get(Servo.class, "claw_back");

    // Sensors
      // bottomColor = hardwareMap.get(ColorSensor.class, "bottom_color");
    backDist = hardwareMap.get(DistanceSensor.class, "back_dist");
    // frontDist = hardwareMap.get(DistanceSensor.class, "front_dist");
      touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");

    // Set Directions
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);

    leftIntake.setDirection(DcMotor.Direction.FORWARD);
    rightIntake.setDirection(DcMotor.Direction.REVERSE);

      // you can also cast this to a Rev2mDistanceSensor if you want to use added methods associated
      // with the Rev2mDistanceSensor class.
    Rev2mDistanceSensor backTOF = (Rev2mDistanceSensor) backDist;

      touchSensor.setMode(DigitalChannel.Mode.INPUT);

      // Set servos to their "0" position
    capstone(false);
     foundationServo(true);
     pusher(true);
     clawServo(true); // close the claw within the robot

    // Set IMU Parameters
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    // Hardware Map IMU
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    // Reset Robot Angle
    resetAngle();

    if (auton) {
      setBrake(true);
      setEncoders(true);
    } else {
    }

    // Gyro Data
    telemetry.addData("Gyro Calibrated", imu.getCalibrationStatus());
    telemetry.addData("Robot", "Ready");
    telemetry.update();
  }

  // moves the servos that pull the foundation
  public void foundationServo(boolean option) {
    if (option) { // open
      foundationLeft.setPosition(0.65);
      foundationMiddle.setPosition(.9);
      foundationRight.setPosition(0);

    } else { // close
      foundationLeft.setPosition(0.0); // add one
       foundationMiddle.setPosition(0.3);
      foundationRight.setPosition(0.65); // subtract one
    }
  }
  
    public void foundationServoDrive(boolean option) {
    if (option) { // open
      foundationLeft.setPosition(0.65);
      //foundationMiddle.setPosition(.9);
      foundationRight.setPosition(0);

    } else { // close
      foundationLeft.setPosition(0.0); // add one
       //foundationMiddle.setPosition(0.3);
      foundationRight.setPosition(0.65); // subtract one
    }
  }
  
    public void pusher(boolean option) {
    if (option) {
      stonePush.setPosition(0.03); // open

    } else {
      stonePush.setPosition(0.9);
    }
  }


    public void capstone(boolean option) {
    if (option) {
      capstoneServo.setPosition(0.9);

    } else {
      capstoneServo.setPosition(0.1);
    }
    }

  public void clawServo(boolean option) {
    if (option) {
      clawFront.setPosition(0.48);
      clawBack.setPosition(0.19);
    } else {
      clawFront.setPosition(0.7);
      clawBack.setPosition(0);
    }
  }

  // Autonomous Functions
  // resets the angle to change it to a 0 heading
  public void resetAngle() {
    lastAngles =
            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    globalAngle = 0;
  }

  // intializes encoders on all the drive motors
  public void setEncoders(boolean run) {

    if (run == true) {
      leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } else {
      leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  }

  // stops the robot so that robot does not go out of control
  public void setBrake(boolean run) {

    if (run == true) {
      leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
  }

  // sets all motor powers to 0
  public void stop() {
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
  }

  // intake direction by typing "in" or "out" and giving it a power
  public void intake(String option, double value) {
    double power = value;
    if (option.equals("in") || option.equals("In") || option.equals("IN")) {
      leftIntake.setPower(-power);
      rightIntake.setPower(-power);
    } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
      leftIntake.setPower(power);
      rightIntake.setPower(power);
    } else {
      leftIntake.setPower(0);
      rightIntake.setPower(0);
    }
  }

  // overrides previous intake function and gives power of 1
  public void intake(String option) {
    double power = 1;
    if (option.equals("in") || option.equals("In") || option.equals("IN")) {
      leftIntake.setPower(-power);
      rightIntake.setPower(-power);
    } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
      leftIntake.setPower(power);
      rightIntake.setPower(power);
    } else {
      leftIntake.setPower(0);
      rightIntake.setPower(0);
    }
  }

    public void tfData(int i, Recognition recognition) {
        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
        telemetry.addData(
                String.format("  left,top (%d)", i),
                "%.03f , %.03f",
                recognition.getLeft(),
                recognition.getTop());
        telemetry.addData(
                String.format("  right,bottom (%d)", i),
                "%.03f , %.03f",
                recognition.getRight(),
                recognition.getBottom());
        telemetry.addData("midX", (recognition.getLeft() + recognition.getRight()) / 2);
    }

    public boolean isTouched() {
        boolean touched;

        if (touchSensor.getState() == true) {
            //telemetry.addData("Touch", "Is Not Pressed");
            touched = false;
        } else {
            // telemetry.addData("Touch", "Is Pressed");
            touched = true;
        }

        return touched;
    }
}
