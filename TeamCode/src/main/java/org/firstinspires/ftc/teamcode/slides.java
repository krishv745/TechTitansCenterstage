package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class slides extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo slides1 = hardwareMap.servo.get("slides1");
        Servo slides2 = hardwareMap.servo.get("slides2");
        double ljspos = gamepad1.left_stick_y;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (ljspos > 0.5) {
                slides1.setPosition(135);
                TimeUnit.MILLISECONDS.sleep(150);
                if (ljspos == 1.0) {
                    slides2.setPosition(135);
                    TimeUnit.MILLISECONDS.sleep(150);
                }
            }
            else if (ljspos < 0.5) {
                slides1.setPosition(0);
                TimeUnit.MILLISECONDS.sleep(150);
                if (ljspos == 0.0) {
                    slides2.setPosition(0);
                    TimeUnit.MILLISECONDS.sleep(150);
                }
            }
            telemetry.addData("slide1pos: ", slides1.getPosition());
            telemetry.addData("slide2pos: ", slides2.getPosition());
            telemetry.addData("jspos: ", ljspos);
            telemetry.update();

        }
    }
}