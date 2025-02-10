package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualTurret;

public class DualSlideSetLengthWithLimit implements Action {
    DualSlideSetLength setLength;
    DualTurret turrets;
    double maxGroundDistance;

    public DualSlideSetLengthWithLimit(DualSlideSetLength setLength, DualTurret turrets, double maxGroundDistance) {
        this.setLength = setLength;
        this.turrets = turrets;
        this.maxGroundDistance = maxGroundDistance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double futureGroundDistance = Math.sin(turrets.getAngleRadians()) * setLength.targetLengthInches;
        if (futureGroundDistance > maxGroundDistance) {
            double originalTarget = setLength.targetLengthInches;
            setLength.targetLengthInches = maxGroundDistance / Math.sin(turrets.getAngleRadians());
            setLength.run(packet);
            setLength.targetLengthInches = originalTarget;
            return setLength.stillRunning();
        } else {
            return setLength.run(packet);
        }
    }
}
