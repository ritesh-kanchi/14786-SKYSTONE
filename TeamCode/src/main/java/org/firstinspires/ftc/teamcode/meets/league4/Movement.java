package org.firstinspires.ftc.teamcode.meets.league4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Movement {

    /* VARIABLES */

    static final double driveClip = 0.6;

    /* OBJECTS */
    HardwareBot robot = null;
    LinearOpMode opmode = null;

    /* CONSTRUCTOR */
    public Movement(HardwareBot arobot, LinearOpMode aopmode) {
        robot = arobot;
        opmode = aopmode;
    }

    /* FUNCTIONS */
    // Global Functions
    // intake direction by typing "in" or "out" and giving it a power
    public void intake(String option, double value) {
        double power = value;
        if (option.equals("in") || option.equals("In") || option.equals("IN")) {
            robot.leftIntake.setPower(-power);
            robot.rightIntake.setPower(-power);
        } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
            robot.leftIntake.setPower(power);
            robot.rightIntake.setPower(power);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    // overrides previous intake function and gives power of 1
    public void intake(String option) {
        double power = 1;
        if (option.equals("in") || option.equals("In") || option.equals("IN")) {
            robot.leftIntake.setPower(-power);
            robot.rightIntake.setPower(-power);
        } else if (option.equals("out") || option.equals("Out") || option.equals("OUT")) {
            robot.leftIntake.setPower(power);
            robot.rightIntake.setPower(power);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    // TeleOp Functions
    // function to move the chassis during TeleOp
    public void mecDrive(double forward, double turn, double strafe, double clip) {
        robot.leftFront.setPower(Range.clip(forward + turn + strafe, -clip, clip));
        robot.rightFront.setPower(Range.clip(forward - turn - strafe, -clip, clip));
        robot.leftBack.setPower(Range.clip(forward + turn - strafe, -clip, clip));
        robot.rightBack.setPower(Range.clip(forward - turn + strafe, -clip, clip));
    }

    public void mecDrive(double forward, double turn, double strafe, boolean clip) {
        if (clip) {
            robot.leftFront.setPower(Range.clip(forward + turn + strafe, -driveClip, driveClip));
            robot.rightFront.setPower(Range.clip(forward - turn - strafe, -driveClip, driveClip));
            robot.leftBack.setPower(Range.clip(forward + turn - strafe, -driveClip, driveClip));
            robot.rightBack.setPower(Range.clip(forward - turn + strafe, -driveClip, driveClip));
        } else {
            robot.leftFront.setPower(forward + turn + strafe);
            robot.rightFront.setPower(forward - turn - strafe);
            robot.leftBack.setPower(forward + turn - strafe);
            robot.rightBack.setPower(forward - turn + strafe);
        }
    }

    // strafe function that uses mecDrive to strafe during TeleOp
    public void strafe(String option, double power) {
        if (option.equals("left") || option.equals("Left")) {
            mecDrive(0, 0, -1, power);
        }

        if (option.equals("right") || option.equals("Right")) {
            mecDrive(0, 0, 1, power);
        }
    }

    // Autonomous Functions
    // keeps robot going in a given heading and corrects as needed
    public double checkDirection(float wantedAngle) {
        double correction, globalAngle, gain = 0.01;
        globalAngle = getGlobalAngle();
        if (globalAngle == wantedAngle) correction = 0;
        else correction = -(globalAngle - wantedAngle);
        correction = correction * gain;

        return correction;
    }


    // returns given angle
    public double getGlobalAngle() {
        Orientation currentAngles =
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = currentAngles.firstAngle - robot.lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        robot.globalAngle += deltaAngle;

        robot.lastAngles = currentAngles;

        return robot.globalAngle;
    }
}
