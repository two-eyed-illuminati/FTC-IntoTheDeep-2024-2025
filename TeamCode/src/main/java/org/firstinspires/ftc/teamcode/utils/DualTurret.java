package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DualTurret {
    public DcMotorEx turretMotor1;
    public DcMotorEx turretMotor2;
    public final double PULSES_PER_REVOLUTION = 5281.1*4; // 4 is gear ratio
    public final double RESTING_ANGLE_RADIANS = Math.PI*115.0/180.0;
    public final double MAX_PULSES = (40.0/360.0)*(5281.1*4);

    public DualTurret(DcMotorEx turretMotor1, DcMotorEx turretMotor2, boolean reset) {
        this.turretMotor1 = turretMotor1;
        if(reset) {
            this.turretMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretMotor1.setTargetPosition(0);
        }
        this.turretMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.turretMotor2 = turretMotor2;
        if(reset) {
            this.turretMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretMotor2.setTargetPosition(0);
        }
        this.turretMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setAngleRadians(double angleRadians, double maxVelocity) {
        // Encoder = 0 is resting position, encoder increases as turret rotates upwards
        angleRadians -= RESTING_ANGLE_RADIANS;

        double angleDegrees = angleRadians * 180 / Math.PI;
        double pulses = angleDegrees * PULSES_PER_REVOLUTION / 360;
        pulses = Math.max(-(1.0/6.0)*(5281.1*4), pulses);
        pulses = Math.min(MAX_PULSES, pulses);
        turretMotor1.setTargetPosition((int) pulses);
        turretMotor2.setTargetPosition((int) pulses);

        maxVelocity = Math.min(maxVelocity, 5281.1*30.0/60.0);
        double m1Error = Math.abs(pulses - turretMotor1.getCurrentPosition());
        double m2Error = Math.abs(pulses - turretMotor2.getCurrentPosition());

        double m1Velocity = maxVelocity*(1+5*(m1Error-m2Error)/PULSES_PER_REVOLUTION);
        m1Velocity = Math.min(m1Velocity, 5281.1*30.0/60.0);
        turretMotor1.setVelocity(m1Velocity);

        double m2Velocity = maxVelocity*(1+5*(m2Error-m1Error)/PULSES_PER_REVOLUTION);
        m2Velocity = Math.min(m2Velocity, 5281.1*30.0/60.0);
        turretMotor2.setVelocity(m2Velocity);
    }
    public void setAngleRadians(double angleRadians){
        setAngleRadians(angleRadians, 100000);
    }
    public void setAngleDegrees(double angleDegrees, double maxVelocity) {
        setAngleRadians(angleDegrees * Math.PI / 180, maxVelocity);
    }
    public void setAngleDegrees(double angleDegrees) {
        setAngleDegrees(angleDegrees, 100000);
    }

    public double getAngleRadians() {
        return turretMotor1.getCurrentPosition() * 2 * Math.PI / PULSES_PER_REVOLUTION + RESTING_ANGLE_RADIANS;
    }
    public double getAngleDegrees() {
        return turretMotor1.getCurrentPosition() * 360 / PULSES_PER_REVOLUTION + RESTING_ANGLE_RADIANS * 180 / Math.PI;
    }
    public double getEncoder(){
        return turretMotor1.getCurrentPosition();
    }
}

