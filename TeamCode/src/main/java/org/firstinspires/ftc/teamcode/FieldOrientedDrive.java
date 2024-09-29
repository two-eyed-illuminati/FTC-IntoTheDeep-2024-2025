package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldOrientedDrive {
    DcMotor fl, fr, bl, br;
    IMU imu;

    public double rotSpeed = 1.0;

    public FieldOrientedDrive(DcMotor _fl, DcMotor _fr, DcMotor _bl, DcMotor _br, IMU _imu){
        fl = _fl;
        fr = _fr;
        bl = _bl;
        br = _br;
        imu = _imu;
    }

    public void drive(double x, double y, double rotation){
        double currRobotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double targetFieldOrientedMoveHeading = Math.atan2(y, x);
        double targetRobotOrientedMoveHeading = targetFieldOrientedMoveHeading - currRobotHeading;

        double targetSpeed = Math.sqrt(x*x + y*y); // TODO: Consider making this sqrt(x^4 + y^4) for easier control at lower speeds
        double targetRobotOrientedXMove = Math.sin(targetRobotOrientedMoveHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetRobotOrientedMoveHeading) * targetSpeed;

        double flSpeed = -targetRobotOrientedYMove - targetRobotOrientedXMove + rotation * rotSpeed;
        double frSpeed = -targetRobotOrientedYMove + targetRobotOrientedXMove - rotation * rotSpeed;
        double blSpeed = -targetRobotOrientedYMove + targetRobotOrientedXMove + rotation * rotSpeed;
        double brSpeed = -targetRobotOrientedYMove - targetRobotOrientedXMove - rotation * rotSpeed;

        double maxSpeed = Math.max(Math.max(Math.abs(flSpeed), Math.abs(frSpeed)), Math.max(Math.abs(blSpeed), Math.abs(brSpeed)));
        if(maxSpeed > 1){
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            blSpeed /= maxSpeed;
            brSpeed /= maxSpeed;
        }

        fl.setPower(flSpeed);
        fr.setPower(frSpeed);
        bl.setPower(blSpeed);
        br.setPower(brSpeed);
    }
}
