package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DualSlideSetLength implements Action {
    DualSlide slides;
    public double targetLengthInches;
    double maxVelocity;
    double margin = 0.25;
    double lastPos = -1000000;
    int count = 0;
    boolean initialized = false;
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
//        if(!initialized){
//            lastPos = slides.getLength();
//        }
//        else{
//            if(stillRunning() && targetLengthInches <= 12 && Math.abs(slides.getLength() - lastPos) < 0.05){
//                count++;
//            }
//            else{
//                lastPos = slides.getLength();
//                count = 0;
//            }
//            if(count >= 10){
//                slides.slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                slides.slideMotor1.setTargetPosition(0);
//                slides.slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slides.slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                count = 0;
//            }
//        }
        slides.setLength(targetLengthInches, maxVelocity);
        return stillRunning();
    }
}
