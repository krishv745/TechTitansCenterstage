package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

/*
NECESITIES - 
Drivetrain - 4 motors
Intake - 1 motor
Rigging - 2 motors, 2 servos
Slides - 4 servos
Outtake - 2 servos
Transfer - 1 servo
Flight Launcher - 1 servo
*/


@TeleOp
public class Ateleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        // DRIVETRAIN
        DcMotor fl = hardwareMap.dcMotor.get("fl"); //port 0 CH
        DcMotor bl = hardwareMap.dcMotor.get("bl"); //port 1 CH
        DcMotor br = hardwareMap.dcMotor.get("br"); //port 2 CH
        DcMotor fr = hardwareMap.dcMotor.get("fr"); //port 3 CH

        DcMotor intake = hardwareMap.dcMotor.get("intake"); //port 0 EH

        DcMotor rr = hardwareMap.dcMotor.get("rr"); // rigging right, port 1 EH
        DcMotor rl = hardwareMap.dcMotor.get("rl"); // rigging left, port 2 EH

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        Servo servoTransfer = hardwareMap.servo.get("transfer");
        Servo servoHL = hardwareMap.servo.get("HookLeft");
        Servo servoHR = hardwareMap.servo.get("HookRight");
        Servo servoSLB = hardwareMap.servo.get("SlidesLeftBottom");
        Servo servoSLT = hardwareMap.servo.get("SlidesLeftTop");
        Servo servoSRT = hardwareMap.servo.get("SlidesRightTop");
        Servo servoSRB = hardwareMap.servo.get("SlidesRightBottom"); // port 1 CH
        Servo servoFlight = hardwareMap.servo.get("Flight");
        Servo servoRR = hardwareMap.servo.get("RightR"); // port 0 CH
        Servo servoRL = hardwareMap.servo.get("LeftR");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y - gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double brPower = (y + x - rx) / denominator;
            double frPower = (y - x - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            br.setPower(brPower);
            fr.setPower(frPower);

            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                intake.setPower(1);
            } else if (gamepad1.right_bumper && gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
}
