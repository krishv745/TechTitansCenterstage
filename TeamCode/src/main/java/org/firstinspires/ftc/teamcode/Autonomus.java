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

@Config
@Autonomous(name = "Autonomus", group = "Autonomous")
public class Autonomus extends LinearOpMode {


    public class Transfer {
        private Servo servoTransfer;

        public Transfer(HardwareMap hardwareMap) {
            servoTransfer = hardwareMap.get(Servo.class, "transfer");
        }

        public class rotateUp implements Action {
            //servoTransfer.setPosition(0.1); // TEMPORARY
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoTransfer.setPosition(0.1);
                return false;
            }
        }

        public Action rotateUp() {
            return new rotateUp();
        }

        public class rotateDown implements Action {
            //servoTransfer.setPosition(0.4); // TEMPORARY
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoTransfer.setPosition(0.9);
                return false;
            }

            public Action rotateDown() {
                return new rotateDown();
            }
        }
    }

    public class servoHL {
        private Servo servoHL; //servo hook left

        public servoHL(HardwareMap hardwareMap) {
            servoHL = hardwareMap.get(Servo.class, "HookLeft");
        }


        public class hookUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoHL.setPosition(0.2);
                return false;
            }
        }

        public Action hookUp() {
            return new hookUp();
        }

        public class hookDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoHL.setPosition(0.4);
                return false;
            }
        }

        public Action hookDown() {
            return new hookDown();
        }
    }

    public class servoHR {
        private Servo servoHR;//servo hook right

        public servoHR(HardwareMap hardwareMap) {
            servoHR = hardwareMap.get(Servo.class, "HookRight");
        }

        public class hookUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoHR.setPosition(0.2);
                return false;
            }
        }

        public Action hookUp() {
            return new hookUp();
        }

        public class hookDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoHR.setPosition(0.4);
                return false;
            }
        }

        public Action hookDown() {
            return new hookDown();
        }
    }

    public class servoSLB {
        private Servo servoSLB;// slides left bottom

        public servoSLB(HardwareMap hardwareMap) {
            servoSLB = hardwareMap.get(Servo.class, "SlidesLeftBottom");
        }

        public class slideUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSLB.setPosition(0.9);
                return false;
            }
        }

        public Action slideUp() {
            return new slideUp();
        }

        public class slideDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSLB.setPosition(0.1);
                return false;
            }
        }

        public Action slideDown() {
            return new slideDown();
        }
    }

    public class servoSLT {
        private Servo servoSLT;// slides left top

        public servoSLT(HardwareMap hardwareMap) {
            servoSLT = hardwareMap.get(Servo.class, "SlidesLeftTop");
        }

        public class slideUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSLT.setPosition(0.9);
                return false;
            }
        }

        public Action slideUp() {
            return new slideUp();
        }

        public class slideDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSLT.setPosition(0.1);
                return false;
            }
        }

        public Action slideDown() {
            return new slideDown();
        }
    }

    public class servoSRT {
        private Servo servoSRT;// slides Right top

        public servoSRT(HardwareMap hardwareMap) {
            servoSRT = hardwareMap.get(Servo.class, "SlidesRightTop");
        }

        public class slideUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSRT.setPosition(0.9);
                return false;
            }
        }

        public Action slideUp() {
            return new slideUp();
        }

        public class slideDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSRT.setPosition(0.1);
                return false;
            }
        }

        public Action slideDown() {
            return new slideDown();
        }
    }

    public class servoSRB {
        private Servo servoSRB;// slides right bottom

        public servoSRB(HardwareMap hardwareMap) {
            servoSRB = hardwareMap.get(Servo.class, "SlidesRightBottom");
        }

        public class slideUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSRB.setPosition(0.9);
                return false;
            }
        }

        public Action slideUp() {
            return new slideUp();
        }

        public class slideDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoSRB.setPosition(0.1);
                return false;
            }
        }

        public Action slideDown() {
            return new slideDown();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(); // not figured out yet
        Transfer transfer = new Transfer(hardwareMap);
        servoHL hookLeft = new servoHL(hardwareMap);
        servoHR hookRight = new servoHR(hardwareMap);
        servoSLB slideLeftBottom = new servoSLB(hardwareMap);
        servoSLT slideLeftTop = new servoSLT(hardwareMap);
        servoSRT slideRightTop = new servoSRT(hardwareMap);
        servoSRB slideRightBottom = new servoSRB(hardwareMap);
    }

}
