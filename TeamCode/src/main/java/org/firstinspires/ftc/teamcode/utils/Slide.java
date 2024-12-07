package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Slide {
    public DcMotorEx slideMotor;
    final double PULSES_PER_INCH = 114.597403; //Encoder:6 Length 9.5, Encoder:2212 Length:28.75
    final double COLLAPSED_LENGTH = 10.0; // Length from pivot point to claw
    final double MAX_PULSES = 2350.0; // Maximum encoder value for fully extended slide

    public Slide(DcMotorEx slideMotor) {
        this.slideMotor = slideMotor;
        this.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideMotor.setTargetPosition(0);
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLength(double lengthInches, double maxVelocity) {
        double pulses = PULSES_PER_INCH * (lengthInches - COLLAPSED_LENGTH);
        pulses = Math.min(pulses, MAX_PULSES);
        pulses = Math.max(pulses, 50);

        slideMotor.setTargetPosition((int) pulses);
        maxVelocity = Math.min(maxVelocity, 537.7*312.0/60.0);
        slideMotor.setVelocity(maxVelocity);
    }
    public void setLength(double lengthInches){
        setLength(lengthInches, 100000);
    }

    public double getLength() {
        return (slideMotor.getCurrentPosition() / PULSES_PER_INCH) + COLLAPSED_LENGTH;
    }

    public double getEncoder(){
        return slideMotor.getCurrentPosition();
    }
}
