package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class slidesTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo ls1 = hardwareMap.servo.get("ls1");
        Servo ls2 = hardwareMap.servo.get("ls2");
        double ls1Pos  = 0;
        double ls2Pos  = 0;
//        ljspos = gamepad1.left_stick_y;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                ls1Pos += 0.1;
                ls2Pos += 0.1;
                ls1.setPosition(ls1Pos);
                ls2.setPosition(ls2Pos);
                TimeUnit.MILLISECONDS.sleep(150);
            } else if (gamepad1.dpad_down) {
                ls1Pos -= 0.1;
                ls2Pos -= 0.1;
                ls1.setPosition(ls1Pos);
                ls2.setPosition(ls2Pos);
                TimeUnit.MILLISECONDS.sleep(150);
            }
            telemetry.addData("slide1pos: ", ls1.getPosition());
            telemetry.addData("slide2pos: ", ls2.getPosition());
            telemetry.update();

        }
    }
}
