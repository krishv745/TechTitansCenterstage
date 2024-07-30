package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//add import for the RobotValues class
import static org.firstinspires.ftc.teamcode.RobotValues.*;


public class RobotDrive {

    DcMotorEx lf; //Extened additional functions: current, velocity
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    
    IMU imu;
    static double headingOffset = 0;
    
    //Hardware map in opcode only
    
    public DcMotorEx initDcMotor(HardwareMap hardwareMap, String name, DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }
    
    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor(hardwareMap, "lf", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "rf", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "lb", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "rb", DcMotor.Direction.FORWARD);
        initIMU(hardwareMap);
    }
    
    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);
    }
        
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
    }

    public double getHeading() {
        return headingOffset + getIMUHeading();
    }
        
    public void driveXYW(double rx, double ry, double rw) {
        // both joysticsk forward. Also used in navagation
        double denom = Math.max(Math.abs(rx)+Math.abs(rx)+Math.abs(rx), 1.0);
        
        double lfPower = rx - ry - rw;
        double rfPower = rx + ry + rw;
        double lbPower = rx + ry - rw;
        double rbPower = rx - ry + rw;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }
    
    public void driveFieldXYW(double fx, double fy, double fw, double rot) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading()) + rot;
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }
}
