package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Slide {
    DcMotorEx slideMotor;
    final double PULSES_PER_REVOLUTION = 5281.1;
    final double MAX_REVOLUTIONS_PER_MIN = 30;
    final double INCHES_EXTENDED_PER_REVOLUTION = -1.0; //Figure out later
    final double COLLAPSED_LENGTH = -1.0; //Figure out later

    public Slide(DcMotorEx slideMotor) {
        this.slideMotor = slideMotor;
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLength(double lengthInches, double maxVelocity) {
        double pulses = (PULSES_PER_REVOLUTION / INCHES_EXTENDED_PER_REVOLUTION) * (lengthInches - COLLAPSED_LENGTH);

        assert(maxVelocity < MAX_REVOLUTIONS_PER_MIN * PULSES_PER_REVOLUTION / 60);
        slideMotor.setVelocity(maxVelocity);
        slideMotor.setTargetPosition((int) pulses);
    }

    public double getLength() {
        return (slideMotor.getCurrentPosition() / PULSES_PER_REVOLUTION) * INCHES_EXTENDED_PER_REVOLUTION + COLLAPSED_LENGTH;
    }
}
