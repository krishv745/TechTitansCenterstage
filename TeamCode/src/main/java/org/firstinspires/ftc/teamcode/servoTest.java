//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.hardware.Servo;
//
//import java.util.concurrent.TimeUnit;
//
//@TeleOp(group = "ZTest")
//public class servoTest extends LinearOpMode {
//    public void runOpMode() throws InterruptedException {
//        String servoName = ""; // add servo name for testing
//        Servo servoTest = hardwareMap.Servo.get(servoName);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        double pos = 0.0;
//
//        while (opModeIsActive) {
//            if (gamepad1.dpad_up && pos < 1.0) {
//                pos += 0.05;
//            } else if (gamepad1.dpad_right && pos < 1.0) {
//                post += 0.01;
//            } else if (gamepad1.dpad_down && pos > 0.0) {
//                pos -= 0.05;
//            } else if (gamepad1.dpad_left && pos > 0.0) {
//                pos -= 0.01;
//            }
//
//            servoTest.setPosition(pos);
//            TimeUnit.MILLISECONDS.sleep(200);
//            telemetry.addData("Position: ", pos);
//            telemetry.update();
//        }
//    }
//}