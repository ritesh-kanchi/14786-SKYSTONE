package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareBot;

@TeleOp(name = "GimmeEncoders")
public class GimmeEncoders extends LinearOpMode {

    HardwareBot robot = new HardwareBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry, true);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setEncoders(true);
        waitForStart();
        while (opModeIsActive()) {

            double lfCurrent = robot.leftFront.getCurrentPosition();
            double rfCurrent = robot.rightFront.getCurrentPosition();
            double lbCurrent = robot.leftBack.getCurrentPosition();
            double rbCurrent = robot.rightBack.getCurrentPosition();

            telemetry.addData("LF given", lfCurrent);

            telemetry.addData("RF given", rfCurrent);

            telemetry.addData("LB given", lbCurrent);

            telemetry.addData("RB given", rbCurrent);

            // DONT REALLY NEED THIS \/ It's there in case you need it

            telemetry.addData("LF converted", convert(lfCurrent));

            telemetry.addData("RF converted", convert(rfCurrent));

            telemetry.addData("LB converted", convert(lbCurrent));

            telemetry.addData("RB converted", convert(rbCurrent));
            telemetry.update();
        }
    }

    public double convert(double convertValue) {
        double newValue = (convertValue * 1) / ((100 * 0.0393701) * 3.1415);
        return newValue;
    }

    public void gap() {
        sleep(500);
    }
}