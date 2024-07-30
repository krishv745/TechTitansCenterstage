package org.firstinspires.ftc.teamcode;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class viperslides extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DcMotorEx motorSlideLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx motorSlideRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        int position = 0;
        int prevposition = 0;
        boolean a = false;
        int speed = 4000;

        while (opModeIsActive()) {

            if (gamepad2.left_stick_y != 0) {
                motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideRight.setVelocity(-signum(gamepad2.left_stick_y) * 1900);
                motorSlideLeft.setVelocity(-signum(gamepad2.left_stick_y) * 2000);
                position = motorSlideLeft.getCurrentPosition();
                prevposition = position;
                a = true;
            } else if (a) {
                motorSlideRight.setVelocity(0);
                motorSlideLeft.setVelocity(0);
                motorSlideRight.setTargetPosition(motorSlideLeft.getCurrentPosition());
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                position = motorSlideLeft.getCurrentPosition();
                prevposition = position;
                a = false;
            }
            if (prevposition != position && gamepad2.left_stick_y == 0) {
                motorSlideRight.setTargetPosition(position);
                motorSlideLeft.setTargetPosition(position);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(speed);
                motorSlideLeft.setVelocity(speed);
                prevposition = position;
            }
            telemetry.addData("position", position);
            telemetry.addData("right", motorSlideRight.getCurrentPosition());
            telemetry.addData("positionReal", motorSlideRight.getTargetPosition());
            telemetry.addData("lefd", motorSlideLeft.getCurrentPosition());

        }
    }
}