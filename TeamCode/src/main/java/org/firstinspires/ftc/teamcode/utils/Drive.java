package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    DcMotorEx fl, fr, bl, br;
    IMU imu;

    public double rotSpeed = 1.0;

    public Drive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, IMU imu){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu = imu;
    }

    public void resetImu(){
        imu.resetYaw();
    }

    public double[] driveNormal(double x, double y, double rotation){
        //Square the distance so that it is easier to control at low speeds
        double targetSpeed = Math.pow(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 2);

        double targetHeading = -Math.atan2(-x, y);
        double targetRobotOrientedXMove = Math.sin(targetHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetHeading) * targetSpeed;

        //Square rotation so that it is easier to control at low rotation speeds
        double correctedSquaredRotation = Math.pow(rotation, 2) * Math.signum(rotation);
        double flSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove + correctedSquaredRotation * rotSpeed;
        double frSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove - correctedSquaredRotation * rotSpeed;
        double blSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove + correctedSquaredRotation * rotSpeed;
        double brSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove - correctedSquaredRotation * rotSpeed;

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

        return new double[]{flSpeed, frSpeed, blSpeed, brSpeed,
                targetSpeed, targetRobotOrientedXMove, targetRobotOrientedYMove};
    }

    public double[] driveFieldCentric(double x, double y, double rotation){
        double currRobotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double targetFieldOrientedMoveHeading = -Math.atan2(-x, y);
        double targetRobotOrientedMoveHeading = targetFieldOrientedMoveHeading - currRobotHeading;

        //Square the distance so that it is easier to control at low speeds
        double targetSpeed = Math.pow(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 2);
        double targetRobotOrientedXMove = Math.sin(targetRobotOrientedMoveHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetRobotOrientedMoveHeading) * targetSpeed;

        //Square rotation so that it is easier to control at low rotation speeds
        double correctedSquaredRotation = Math.pow(rotation, 2) * Math.signum(rotation);
        double flSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove + correctedSquaredRotation * rotSpeed;
        double frSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove - correctedSquaredRotation * rotSpeed;
        double blSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove + correctedSquaredRotation * rotSpeed;
        double brSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove - correctedSquaredRotation * rotSpeed;

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

        return new double[]{flSpeed, frSpeed, blSpeed, brSpeed,
                targetFieldOrientedMoveHeading, targetRobotOrientedMoveHeading,
                targetSpeed, targetRobotOrientedXMove, targetRobotOrientedYMove,
                currRobotHeading};
    }
}
