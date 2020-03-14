package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "HDriveTest")
public class HDriveTest extends LinearOpMode {

    // create motor vars
    private DcMotor leftDrive, rightDrive, centerDrive;
    // create gyro vars
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    @Override
    public void runOpMode() {

        // hardware maps
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");

        // set directions
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // set 0 powers - BRAKE=HARD STOP, FLOAT=FREESPIN
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reset encoders to 0
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to use encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set up the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // hardware maps for gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // set robot angle to 0
        resetAngle();

        // wait for start
        waitForStart();
        // motions
        // power, distance, angle (should be 0 without turns)
        forward(0.6, 20, 0);
        left(0.6, 10, 0);
        right(0.6, 10, 0);
        backward(0.6, 10, 0);
    }

    // these are called functions, they allow us to minimize code available to shorten files and
    // organize and stuff
    // resets the angle to change it to 0 degrees
    public void resetAngle() {
        lastAngles =
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    // checks the current direction
    public double checkDirection(float wantedAngle) {
        double correction,
                globalAngle,
                gain = 0.01; // make smaller if it goes out of control or recorrects too fast
        globalAngle = getGlobalAngle();
        if (globalAngle == wantedAngle) correction = 0;
        else correction = -(globalAngle - wantedAngle);
        correction = correction * gain;

        return correction;
    }

    // gives us the current angle the robot is facing
    public double getGlobalAngle() {
        Orientation currentAngles =
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = currentAngles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = currentAngles;

        return globalAngle;
    }

    // encoders forward function
    public void forward(double power, double distance, float wantedAngle) {

        // set 0 power
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // create left and right distance vars and correction
        int leftDist, rightDist;
        double correction;

        // multiply by ticks per inch
        int moveTicks =
                (int) (distance * 31.2); // 31.2 ticks = 1 inch, may need to change based on your robot

        // add this value to where the current
        leftDist = leftDrive.getCurrentPosition() + moveTicks;
        rightDist = rightDrive.getCurrentPosition() + moveTicks;

        // set the target, where it wants to go
        leftDrive.setTargetPosition(leftDist);
        rightDrive.setTargetPosition(rightDist);

        // tells motors to run there
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // adds power to drive
        power = Math.abs(power);
        leftDrive.setPower(power);
        rightDrive.setPower(power);

        // how long the motors should run
        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {

            // find how much we should correct by
            correction = checkDirection(wantedAngle);

            // corrections
            leftDrive.setPower(power - correction);
            rightDrive.setPower(power + correction);
            // set to 0 so it can spin freely
            centerDrive.setPower(0);
        }

        setZero(); // set all power to 0 (stop)
        // turn off run to position by run using encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100); // 1/10 sec break
    }

    // backewards, same as above but reversing correction and moveticks
    public void backward(double power, double distance, float wantedAngle) {

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int leftDist, rightDist;
        double correction;

        int moveTicks = (int) (distance * 31.2);

        leftDist = leftDrive.getCurrentPosition() - moveTicks;
        rightDist = rightDrive.getCurrentPosition() - moveTicks;

        leftDrive.setTargetPosition(leftDist);
        rightDrive.setTargetPosition(rightDist);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Math.abs(power);
        leftDrive.setPower(power);
        rightDrive.setPower(power);

        // how long the motors should run
        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {

            // find how much we should correct by
            correction = checkDirection(wantedAngle);

            // corrections
            leftDrive.setPower(power + correction);
            rightDrive.setPower(power - correction);
            centerDrive.setPower(0);
        }

        setZero();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
    }

    // strafe left function, might be right idk
    public void left(double power, double distance, float wantedAngle) {

        // set 0 power behaviors, all to break, so the omni wheels can glide and not spin, causing curve
        leftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE); // might need to change to FLOAT
        rightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE); // might need to change to FLOAT
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int centerDist;
        double correction;

        int moveTicks = (int) (distance * 31.2); // may need to change 31.2

        centerDist = centerDrive.getCurrentPosition() + moveTicks;

        centerDrive.setTargetPosition(centerDist);

        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Math.abs(power);
        centerDrive.setPower(power);

        // how long the motors should run
        while (opModeIsActive()
                && (leftDrive.isBusy() && rightDrive.isBusy() && centerDrive.isBusy())) {

            // find how much we should correct by
            correction = checkDirection(wantedAngle);

            // give power to center wheel, only correction to the outsides, may need to swap +/-
            leftDrive.setPower(+correction);
            rightDrive.setPower(-correction);
            centerDrive.setPower(power);
        }

        setZero();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
    }

    // same as above, but reverse
    public void right(double power, double distance, float wantedAngle) {

        leftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE); // might need to change to FLOAT
        rightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE); // might need to change to FLOAT
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int centerDist;
        double correction;

        int moveTicks = (int) (distance * 31.2);

        centerDist = centerDrive.getCurrentPosition() - moveTicks;

        centerDrive.setTargetPosition(centerDist);

        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Math.abs(power);
        centerDrive.setPower(power);

        // how long the motors should run
        while (opModeIsActive()
                && (leftDrive.isBusy() && rightDrive.isBusy() && centerDrive.isBusy())) {

            // find how much we should correct by
            correction = checkDirection(wantedAngle);

            // give power to center wheel, only correction to the outsides, may need to swap +/-
            leftDrive.setPower(-correction);
            rightDrive.setPower(+correction);
            centerDrive.setPower(power);
        }

        setZero();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
    }

    // sets everything to 0
    public void setZero() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        centerDrive.setPower(0);
    }
}
