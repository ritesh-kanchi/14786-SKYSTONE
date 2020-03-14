package org.firstinspires.ftc.teamcode.meets.league3;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
  // Servos
  public Servo foundationLeft, foundationRight;
  public Servo capstoneServo;
  public Servo intakeDrop;
  // Sensors
  public ColorSensor colorSensor;
  public DistanceSensor distanceSensor;

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

    // Servos
    foundationLeft = hardwareMap.get(Servo.class, "foundation_left");
    foundationRight = hardwareMap.get(Servo.class, "foundation_right");
     capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");
    intakeDrop = hardwareMap.get(Servo.class, "intake_drop");

    // Sensors
    //   colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    // distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

    // Set Directions
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);

    leftIntake.setDirection(DcMotor.Direction.FORWARD);
    rightIntake.setDirection(DcMotor.Direction.REVERSE);

    // you can also cast this to a Rev2mDistanceSensor if you want to use added
    // methods associated with the Rev2mDistanceSensor class.
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;

    // Set to foundation servos to 0

     capstone(false);

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
      intakeDrop(false);
    }

    // Gyro Data
    telemetry.addData("Gyro Calibrated", imu.getCalibrationStatus());
    telemetry.addData("Robot", "Ready");
    telemetry.update();
  }

  // moves the servo that locks the intake before the match
  public void intakeDrop(boolean option) {
    if (option) {
      intakeDrop.setPosition(0); // drop
    } else {
      intakeDrop.setPosition(0.7); // lock
    }
  }

  // moves the servos that pull the foundation
  public void foundationServo(boolean option) {
    if (option) {
      foundationLeft.setPosition(0);
      foundationRight.setPosition(0.35);
    } else {
      foundationLeft.setPosition(0.7);
      foundationRight.setPosition(0);
    }
  }

  public void capstone(boolean option) {
    if (option) {
      capstoneServo.setPosition(0.7);
    } else {
      capstoneServo.setPosition(0.35);
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
  public void setZero() {
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
  }
}