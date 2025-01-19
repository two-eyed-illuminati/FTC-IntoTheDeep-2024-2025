package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DualSlideSetLength implements Action {
    DualSlide slides;
    double targetLengthInches;
    double maxVelocity;
    double margin = 0.5;
    public DualSlideSetLength(DualSlide slides, double targetLengthInches, double maxVelocity){
        this.slides = slides;
        this.targetLengthInches = targetLengthInches;
        this.maxVelocity = maxVelocity;
    }
    public DualSlideSetLength(DualSlide slides, double targetLengthInches){
        this(slides, targetLengthInches, 1000);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet){
        slides.setLength(targetLengthInches, maxVelocity);
        return Math.abs(slides.getLength() - targetLengthInches) > margin;
    }
}
