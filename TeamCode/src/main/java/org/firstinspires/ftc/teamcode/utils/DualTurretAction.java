package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DualTurretAction implements Action {
    DualTurret turret;
    public enum Mode{
        GO_TO,
        GO_ABOVE,
        GO_BELOW
    }
    Mode mode;
    double targetAngleRadians;
    double maxVelocity;
    double margin;

    public DualTurretAction(DualTurret turret) {
        this.turret = turret;
        this.mode = Mode.GO_TO;
        this.targetAngleRadians = turret.getAngleRadians();
        this.maxVelocity = 5281.1*30.0/60.0;
        this.margin = Math.PI * 2 / 180;
    }
    //Use a builder pattern
    //Allows for lines like this: new DualTurretAction(turrets).setTargetAngleRadians(0).setMode(Mode.GO_ABOVE);
    public DualTurretAction setMode(Mode mode){
        this.mode = mode;
        return this;
    }
    public DualTurretAction setTargetAngleRadians(double targetAngleRadians){
        this.targetAngleRadians = targetAngleRadians;
        return this;
    }
    public DualTurretAction setMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
        return this;
    }
    public DualTurretAction setMargin(double margin){
        this.margin = margin;
        return this;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(mode.equals(Mode.GO_TO)) {
            turret.setAngleRadians(targetAngleRadians, maxVelocity);
            return Math.abs(turret.getAngleRadians() - targetAngleRadians) > margin;
        }
        if(mode.equals(Mode.GO_ABOVE)){
            turret.setAngleRadians(100000, maxVelocity);
            return (turret.getAngleRadians() - targetAngleRadians) <= 0;
        }
        if(mode.equals(Mode.GO_BELOW)){
            turret.setAngleRadians(-100000, maxVelocity);
            return (turret.getAngleRadians() - targetAngleRadians) >= 0;
        }
        return false;
    }
}

