package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.jetbrains.annotations.NotNull;


@Autonomous(name = "Autonomus", group = "Autonomous")
public class Autonomus extends LinearOpMode{


    public class Transfer {
        private Servo servoTransfer;
        public Transfer(HardwareMap hardwareMap) {
            servoTransfer = hardwareMap.get(Servo.class, "transfer");
        }

        @Override
        public class rotateUp(@NotNull TelemetryPacket packet) {
            servoTransfer.setPosition(0.1);
        }
    }

    public class servoHL {
        private Servo servoHL; //servo hook left
        public servoHL(HardwareMap hardwareMap) {
            servoHL = hardwareMap.get(Servo.class, "HookLeft");
        }
    }

    public class servoHR {
        private Servo servoHR;//servo hook right
        public servoHR(HardwareMap hardwareMap) {
            servoHR = hardwareMap.get(Servo.class, "HookRight");
        }
    }

    public class servoSLB {
        private Servo servoSLB;// slides left bottom
        public servoSLB(HardwareMap hardwareMap) {
            servoSLB = hardwareMap.get(Servo.class, "SlidesLeftBottom");
        }
    }

    public class servoSLT {
        private Servo servoSLT;// slides left top
        public servoSLT(HardwareMap hardwareMap) {
            servoSLT = hardwareMap.get(Servo.class, "SlidesLeftTop");
        }
    }

    public class servoSRT {
        private Servo servoSRT;// slides Right top
        public servoSRT(HardwareMap hardwareMap) {
            servoSRT = hardwareMap.get(Servo.class, "SlidesRightTop");
        }
    }

    public class servoSRB {
        private Servo servoSRB;// slides right bottom
        public servoSRB(HardwareMap hardwareMap) {
            servoSRB = hardwareMap.get(Servo.class, "SlidesRightBottom");
        }
    }



}
