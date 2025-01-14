package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DualTurretSetAngle implements Action {
    DualTurret turret;
    double targetAngleRadians;
    double maxVelocity;
    double margin = Math.PI * 2 / 180;

    public DualTurretSetAngle(DualTurret turret, double targetAngleRadians, double maxVelocity) {
        this.turret = turret;
        this.targetAngleRadians = targetAngleRadians;
        this.maxVelocity = maxVelocity;
    }

    public DualTurretSetAngle(DualTurret turret, double targetAngleRadians) {
        this(turret, targetAngleRadians, 100000);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        turret.setAngleRadians(targetAngleRadians, maxVelocity);
        return Math.abs(turret.getAngleRadians() - targetAngleRadians) < margin;
    }
}
