package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    public DcMotorEx turretMotor;
    final double PULSES_PER_REVOLUTION = 5281.1*4; // 4 is gear ratio
    final double MAX_REVOLUTIONS_PER_MIN = 30;
    final double RESTING_ANGLE_RADIANS = Math.PI/4; // TODO: Probably Wrong
    final double MAX_PULSES = 7900;

    public Turret(DcMotorEx turretMotor) {
        this.turretMotor = turretMotor;
        this.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turretMotor.setTargetPosition(0);
        this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setAngleRadians(double angleRadians, double maxVelocity) {
        // Encoder = 0 is resting position, encoder increases as turret rotates upwards
        angleRadians -= RESTING_ANGLE_RADIANS;

        double angleDegrees = angleRadians * 180 / Math.PI;
        double pulses = angleDegrees * PULSES_PER_REVOLUTION / 360;
        pulses = Math.max(20, pulses);
        pulses = Math.min(MAX_PULSES, pulses);

        turretMotor.setVelocity(maxVelocity);
        turretMotor.setTargetPosition((int) pulses);
    }
    public void setAngleDegrees(double angleDegrees, double maxVelocity) {
        setAngleRadians(angleDegrees * Math.PI / 180, maxVelocity);
    }

    public double getAngleRadians() {
        return turretMotor.getCurrentPosition() * 2 * Math.PI / PULSES_PER_REVOLUTION + RESTING_ANGLE_RADIANS;
    }
    public double getAngleDegrees() {
        return turretMotor.getCurrentPosition() * 360 / PULSES_PER_REVOLUTION + RESTING_ANGLE_RADIANS * 180 / Math.PI;
    }
}
