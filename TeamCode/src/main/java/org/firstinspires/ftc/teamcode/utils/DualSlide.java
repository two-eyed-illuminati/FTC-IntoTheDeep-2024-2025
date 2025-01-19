package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DualSlide {
    public DcMotorEx slideMotor1;
    public DcMotorEx slideMotor2;
    public final double PULSES_PER_INCH = (2208.0)/(36.75-10);
    public final double COLLAPSED_LENGTH = 10.0; // Length from pivot point to claw
    public final double MAX_PULSES = 2208.0; // Maximum encoder value for fully extended slide
    public DualSlide(DcMotorEx slideMotor1, DcMotorEx slideMotor2) {
        this.slideMotor1 = slideMotor1;
        this.slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideMotor1.setTargetPosition(0);
        this.slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.slideMotor2 = slideMotor2;
        this.slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideMotor2.setTargetPosition(0);
        this.slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLength(double lengthInches, double maxVelocity) {
        double pulses = PULSES_PER_INCH * (lengthInches - COLLAPSED_LENGTH);
        pulses = Math.min(pulses, MAX_PULSES);
        pulses = Math.max(pulses, 50);

        slideMotor1.setTargetPosition((int) pulses);
        slideMotor2.setTargetPosition((int) pulses);
        maxVelocity = Math.min(maxVelocity, 0.9*384.5*435.0/60.0);
        double m1Error = Math.abs(pulses - slideMotor1.getCurrentPosition());
        double m2Error = Math.abs(pulses - slideMotor2.getCurrentPosition());

        double m1Velocity = maxVelocity*(1+0.1*(m1Error-m2Error)/PULSES_PER_INCH);
        m1Velocity = Math.min(m1Velocity, 384.5*435.0/60.0);
        slideMotor1.setVelocity(m1Velocity);

        double m2Velocity = maxVelocity*(1+0.1*(m2Error-m1Error)/PULSES_PER_INCH);
        m2Velocity = Math.min(m2Velocity, 384.5*435.0/60.0);
        slideMotor2.setVelocity(m2Velocity);
    }
    public void setLength(double lengthInches){
        setLength(lengthInches, 100000);
    }

    public double getLength() {
        return (slideMotor1.getCurrentPosition() / PULSES_PER_INCH) + COLLAPSED_LENGTH;
    }

    public double getEncoder(){
        return slideMotor1.getCurrentPosition();
    }
}