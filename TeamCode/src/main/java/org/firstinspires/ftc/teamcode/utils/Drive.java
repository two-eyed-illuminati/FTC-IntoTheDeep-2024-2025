package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    public DcMotorEx fl, fr, bl, br;
    public IMU imu;
    ControlsToValues ctv;

    //TODO increase when aashil gets better at driving
    public double rotSpeed = 0.6;
    public double moveSpeed = 0.6;
    public double targetHeading;

    public Drive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, IMU imu){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.imu = imu;
        this.ctv = new ControlsToValues();
        this.targetHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetImu(){
        imu.resetYaw(); this.targetHeading = 0;
    }

    public double[] driveNormal(double x, double y, double rotation, ControlsToValues ctv){
        double targetSpeed = ctv.targetSpeedFromJoysticks(x, y);

        double targetHeading = -Math.atan2(-x, y);
        double targetRobotOrientedXMove = Math.sin(targetHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetHeading) * targetSpeed;

        double targetRotSpeed = ctv.targetSpeedFromJoysticks(rotation)*Math.signum(rotation);

        double flSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove + targetRotSpeed * rotSpeed;
        double frSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove - targetRotSpeed * rotSpeed;
        double blSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove + targetRotSpeed * rotSpeed;
        double brSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove - targetRotSpeed * rotSpeed;

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

    public double[] driveNormal(double x, double y, double rotation){
        return driveNormal(x, y, rotation, ctv);
    }

    public double[] driveFieldCentric(double x, double y, double rotation, ControlsToValues ctv){
        double currRobotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double targetSpeed = ctv.targetSpeedFromJoysticks(x, y);

        double targetFieldOrientedMoveHeading = -Math.atan2(-x, y);
        double targetRobotOrientedMoveHeading = targetFieldOrientedMoveHeading - currRobotHeading;

        double targetRobotOrientedXMove = Math.sin(targetRobotOrientedMoveHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetRobotOrientedMoveHeading) * targetSpeed;

        double targetRotSpeed = 0;
        if(Math.abs(rotation) < 0.05){
            targetRotSpeed = (targetHeading - currRobotHeading);
        }else{
            targetHeading = currRobotHeading;
            targetRotSpeed = ctv.targetSpeedFromJoysticks(rotation)*Math.signum(rotation)*rotSpeed;
        }

        double flSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove + targetRotSpeed;
        double frSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove - targetRotSpeed;
        double blSpeed = targetRobotOrientedYMove - targetRobotOrientedXMove + targetRotSpeed;
        double brSpeed = targetRobotOrientedYMove + targetRobotOrientedXMove - targetRotSpeed;

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

    public double[] driveFieldCentric(double x, double y, double rotation){
        return driveFieldCentric(x, y, rotation, ctv);
    }
}
