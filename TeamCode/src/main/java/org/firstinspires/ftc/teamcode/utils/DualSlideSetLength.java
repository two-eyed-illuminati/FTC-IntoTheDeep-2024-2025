package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DualSlideSetLength implements Action {
    DualSlide slides;
    public double targetLengthInches;
    double maxVelocity;
    double margin = 0.25;
    public DualSlideSetLength(DualSlide slides, double targetLengthInches, double maxVelocity){
        this.slides = slides;
        this.targetLengthInches = targetLengthInches;
        this.maxVelocity = maxVelocity;
    }
    public DualSlideSetLength(DualSlide slides, double targetLengthInches){
        this(slides, targetLengthInches, 100000);
    }
    public boolean stillRunning(){
        return Math.abs(slides.getLength() - targetLengthInches) > margin;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket packet){
        packet.put("len", slides.getLength());
        packet.put("lenErr", Math.abs(slides.getLength() - targetLengthInches));
        slides.setLength(targetLengthInches, maxVelocity);
        return stillRunning();
    }
}
