package org.firstinspires.ftc.teamcode.league1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareRobot {
  // public vars
  public static final double countsPerRev = 537.6; // testing, value: 560
  public static final double wheelDia =
      100 * 0.0393701; // converting diameter from 100mm to inches, 0.0393701: conversion factor
  // fudge factor for driving forward,backward
  public static final double fudge = 0.512703297598493;
  // fudge factor for strafe
  public static final double fudgeS = 0.66743632519195; // maybe 0.283660438206579
  // inchCount for forward, backward
  public static final double inchCount =
      (countsPerRev * fudge) / (wheelDia * 3.1415); // (counts*fudge)/circumference
  // inchCount for strafe
  public static final double inchCountS =
      (countsPerRev * fudgeS) / (wheelDia * 3.1415); // (counts*fudge)/circumference
  // main hardware: motors, servos, sensors
  public DcMotor leftFront, rightFront, leftBack, rightBack, leftIntake, rightIntake;
  public Servo leftArm, rightArm;
  public BNO055IMU imu;

  // init hardware map
  HardwareMap hwMap = null;

  // constructor
  public HardwareRobot() {}

  // init for the gyro and imu
  public void initGyro(HardwareMap ahwMap) {
    hwMap = ahwMap;
    BNO055IMU.Parameters params = new BNO055IMU.Parameters();
    params.mode = BNO055IMU.SensorMode.IMU;
    params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    params.loggingEnabled = false;

    imu = hwMap.get(BNO055IMU.class, "imu");

    imu.initialize(params);
  }

  // init main hardware mapping: motors, servos
  public void init(HardwareMap ahwMap) {
    // Save reference to Hardware map
    hwMap = ahwMap;

    // Define and Initialize Motors
    leftFront = hwMap.get(DcMotor.class, "left_front");
    rightFront = hwMap.get(DcMotor.class, "right_front");
    leftBack = hwMap.get(DcMotor.class, "left_back");
    rightBack = hwMap.get(DcMotor.class, "right_back");

    leftIntake = hwMap.get(DcMotor.class, "left_intake");
    rightIntake = hwMap.get(DcMotor.class, "right_intake");

    leftArm = hwMap.get(Servo.class, "left_arm");
    rightArm = hwMap.get(Servo.class, "right_arm");

    // set directions of motors
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);

    leftIntake.setDirection(DcMotor.Direction.FORWARD);
    rightIntake.setDirection(DcMotor.Direction.REVERSE);

    // set all motors to zero power
    leftFront.setPower(0);
    leftBack.setPower(0);
    rightFront.setPower(0);
    rightBack.setPower(0);

    leftIntake.setPower(0);
    rightIntake.setPower(0);

    // set non-encoder motors
    leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // set encoder motors
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  // init and reset encoders
  public void initEncoders() {

    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  // bring left arm down
  public void downLeft() {
    leftArm.setPosition(0.36);
  }

  // bring left arm up
  public void upLeft() {
    leftArm.setPosition(-0.5);
  }

  // bring right arm down
  public void downRight() {
    rightArm.setPosition(0.51);
  }

  // bring right arm up
  public void upRight() {
    rightArm.setPosition(0.85);
  }
}
