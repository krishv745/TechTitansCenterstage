package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class motorTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("m");
        waitForStart();
        if (isStopRequested()) return;
        double pwr = 0 ;
        double save = 0;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pwr += 0.1;
            } else if (gamepad1.dpad_down) {
                pwr -= 0.1;
            } else if (gamepad1.a) {
                pwr = 1;
            } else if (gamepad1.b) {
                pwr = 0;
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.x) {
                save = pwr;
            } else if (gamepad1.y) {
                pwr = save;
            }
            motor.setPower(pwr);
            TimeUnit.MILLISECONDS.sleep(100);
            telemetry.addData("Power: ", pwr);
            telemetry.update();
        }
    }
}