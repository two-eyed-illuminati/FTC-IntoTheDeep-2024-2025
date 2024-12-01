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

    public double targetSpeedFromJoysticks(double x, double y){
        //TODO: test new formula
        double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double f = 1/(1 + Math.pow(Math.E, -8.5 * Math.abs(distance) + 3));
        //Normalize F so that normalized_f(0) = 0 and normalized_f(1) = 1
        double normalized_f = (1.0/0.946514325331)*(f-0.0474258731776);
        return normalized_f;
    }
    public double targetSpeedFromJoysticks(double x){
        return targetSpeedFromJoysticks(x, 0);
    }

    public double[] driveNormal(double x, double y, double rotation){
        //Square the distance so that it is easier to control at low speeds
        double targetSpeed = targetSpeedFromJoysticks(x, y);

        double targetHeading = -Math.atan2(-x, y);
        double targetRobotOrientedXMove = Math.sin(targetHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetHeading) * targetSpeed;

        double targetRotSpeed = targetSpeedFromJoysticks(rotation);

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

    public double[] driveFieldCentric(double x, double y, double rotation){
        double currRobotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double targetSpeed = targetSpeedFromJoysticks(x, y);

        double targetFieldOrientedMoveHeading = -Math.atan2(-x, y);
        double targetRobotOrientedMoveHeading = targetFieldOrientedMoveHeading - currRobotHeading;

        double targetRobotOrientedXMove = Math.sin(targetRobotOrientedMoveHeading) * targetSpeed;
        double targetRobotOrientedYMove = Math.cos(targetRobotOrientedMoveHeading) * targetSpeed;

        double targetRotSpeed = targetSpeedFromJoysticks(rotation);

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
                targetFieldOrientedMoveHeading, targetRobotOrientedMoveHeading,
                targetSpeed, targetRobotOrientedXMove, targetRobotOrientedYMove,
                currRobotHeading};
    }
}
