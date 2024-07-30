package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@TeleOp

public class ATeleOp extends LinearOpMode {

    public Minibot bot; //Not robot drive because its connected to minibot
    public DualPad gpad;
    
    @Override
    public void runOpMode() {
        bot = new Minibot(); //Not robot drive because its connected to minibot
        //on the initialization the ATeleop will call the minibot initilization
        //  and the minibot initilization will call the robot drive initilization
        bot.init(hardwareMap);
        
        bot.setManualExposure(this, 10, 120);
        
        gpad = new DualPad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        Minibot.Pose targetPose = new Minibot.Pose(24, 0);

        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            bot.updateTracking();
            telemetry.addData("field pose", bot.field);
            telemetry.addData("targetPose", targetPose);
            double jx = -gpad.left_stick_y - gpad.right_stick_y;
            double jy = -gpad.left_stick_x;
            double jw = -gpad.right_stick_x;
            
            if (gpad.start) {
                if (gpad.dpad_up) bot.setHeading(0);
                if (gpad.dpad_right) bot.setHeading(270);
                if (gpad.dpad_down) bot.setHeading(180);
                if (gpad.dpad_left) bot.setHeading(90);
            }

            int targetId = -1;
            if (gpad.x) targetId = 4;
            if (gpad.y) targetId = 5;
            if (gpad.b) targetId = 6;
            
            List<AprilTagDetection> currentDetections =
                    bot.aprilTag.getDetections();
            boolean tagSeen = false;
            for (AprilTagDetection detection : currentDetections) {
                tagSeen = true;
                if (detection.id == targetId) {
                    jw = detection.ftcPose.bearing * 0.02;
                    jx = (detection.ftcPose.range - 10) * 0.02;
                }
                telemetry.addData("tag", bot.format(detection));
            }
            bot.ledg.enable(tagSeen);
            
            // use current pose as new target
            if (gpad.shift.back) {
                targetPose = new Minibot.Pose(bot.field);
            }
            
            if (gpad.back) {
                bot.driveToPose(targetPose, 0.3);
            }
            else {
                bot.driveXYW(jx, jy, jw);
            }
            // for blue the joystick is being rotated 90 degrees
            //bot.driveFieldXYW(jx, jy, jw, -90);// for red the joystick is being rotated 270 degrees/-90 degrees

            telemetry.addData("Status", "Running");
            telemetry.addData("heading", bot.getHeading());
            telemetry.addData("IMU heading", bot.getIMUHeading());
            telemetry.update();
        }
    }
}
