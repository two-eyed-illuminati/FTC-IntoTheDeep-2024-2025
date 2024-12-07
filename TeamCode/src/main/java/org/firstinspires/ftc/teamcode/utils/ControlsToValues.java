package org.firstinspires.ftc.teamcode.utils;

public class ControlsToValues {
    public double cubicLowerSpeedValue = 0.4;
    public enum Type{
        LINEAR,
        CUBIC
    }
    public Type type = Type.CUBIC;
    public double targetSpeedFromJoysticks(double x, double y){
        if(type == Type.LINEAR){
            return Math.sqrt(x*x + y*y);
        }
        else if(type == Type.CUBIC) {
            double distance = Math.sqrt(x * x + y * y);
            double f;
            f = Math.pow(Math.abs(distance) - 0.5, 3);

            double f0 = Math.pow(Math.abs(0) - 0.5, 3);
            double f1 = Math.pow(Math.abs(1) - 0.5, 3);

            if (Math.abs(distance) >= 0.5) {
                return f * (1 - cubicLowerSpeedValue) / f1 + cubicLowerSpeedValue;
            } else {
                return -f * cubicLowerSpeedValue / f0 + cubicLowerSpeedValue;
            }
        }
        return -1;
    }
    public double targetSpeedFromJoysticks(double x){
        return targetSpeedFromJoysticks(x, 0);
    }

    public double targetVelocityFromJoystick(double x){
        return targetSpeedFromJoysticks(x)*Math.signum(x);
    }
}
