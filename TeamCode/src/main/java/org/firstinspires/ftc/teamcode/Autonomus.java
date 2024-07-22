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

    // SERVOS
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

    public class servoFlight {
        private Servo servoFlight; // for plane launcher

        public servoFlight(HardwareMap hardwareMap) {
            servoFlight = hardwareMap.get(Servo.class, "Flight");
        }

        public class flightUp implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoFlight.setPosition(0.7);
                return false;
            }
        }

        public Action flightUp() {
            return new flightUp();
        }

        public class flightDown implements Action {
            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                servoFlight.setPosition(0.3);
                return false;
            }
        }

        public Action flightDown() {
            return new flightDown();
        }
    }

    // DRIVE MOTORS

    public class Intake {
        private DcMotorEx intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "m");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class intakeOn implements Action {
            @Override
            public boolean run (@NotNull TelemetryPacket packet) {
                intake.setPower(1.0);
                return false;
            }
        }

        public Action intakeOn() {
            return new intakeOn();
        }

        public class intakeOff implements Action {
            @Override
            public boolean run (@NotNull TelemetryPacket packet) {
                intake.setPower(0.0);
                return false;
            }
        }

        public Action intakeOff() {
            return new intakeOff();
        }

        public class intakeReverse implements Action {
            @Override
            public boolean run (@NotNull TelemetryPacket packet) {
                intake.setPower(-1.0);
                return false;
            }
        }

        public Action intakeReverse() {
            return new intakeReverse();
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
        servoFlight flightLauncher = new servoFlight(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        //vision - apriltags, outputs position
        int visionOutputPosition = 1;
        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut; // strafes to an X and Y to get out of a potential partner’s way

        // actionBuilder builds from the drive steps passed to it, 
        // and .build(); is needed to build the trajectory
        trajectoryAction1 = drive.actionBuilder(drive.pose)
            .lineToYSplineHeading(33, Math.toRadians(0))
            .waitSeconds(2)
            .setTangent(Math.toRadians(90))
            .lineToY(48)
            .setTangent(Math.toRadians(0))
            .lineToX(32)
            .strafeTo(new Vector2d(44.5, 30))
            .turn(Math.toRadians(180))
            .lineToX(47.5)
            .waitSeconds(3)
            .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
            .lineToY(37)
            .setTangent(Math.toRadians(0))
            .lineToX(18)
            .waitSeconds(3)
            .setTangent(Math.toRadians(0))
            .lineToXSplineHeading(46, Math.toRadians(180))
            .waitSeconds(3)
            .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
            .lineToYSplineHeading(33, Math.toRadians(180))
            .waitSeconds(2)
            .strafeTo(new Vector2d(46, 30))
            .waitSeconds(3)
            .build();
        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
            .strafeTo(new Vector2d(48, 12))
            .build();

        Actions.runBlocking(flightLauncher.flightDown()); // ON INIT


        while(!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction2;
        } else {
            trajectoryActionChosen = trajectoryAction3;
        }
        
        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionChosen,
                intake.intakeOn(),
                trajectoryAction
            )
        );
    }

}
