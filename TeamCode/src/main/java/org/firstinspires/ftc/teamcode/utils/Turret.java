package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    DcMotorEx turretMotor;
    final double PULSES_PER_REVOLUTION = 5281.1*4; // 4 is gear ratio
    final double MAX_REVOLUTIONS_PER_MIN = 30;

    public Turret(DcMotorEx turretMotor) {
        this.turretMotor = turretMotor;
        this.turretMotor.setTargetPosition(0);
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAngleRadians(double angleRadians, double maxVelocity) {
        // Angle = 0 is resting position, angle increases as turret rotates upwards
        double angleDegrees = angleRadians * 180 / Math.PI;
        double pulses = angleDegrees * PULSES_PER_REVOLUTION / 360;

        assert(maxVelocity < MAX_REVOLUTIONS_PER_MIN * PULSES_PER_REVOLUTION / 60);
        turretMotor.setVelocity(maxVelocity);
        turretMotor.setTargetPosition((int) pulses);
    }
    public void setAngleDegrees(double angleDegrees, double maxVelocity) {
        setAngleRadians(angleDegrees * Math.PI / 180, maxVelocity);
    }

    public double getAngleRadians() {
        return turretMotor.getCurrentPosition() * 2 * Math.PI / PULSES_PER_REVOLUTION;
    }
    public double getAngleDegrees() {
        return turretMotor.getCurrentPosition() * 360 / PULSES_PER_REVOLUTION;
    }
}
