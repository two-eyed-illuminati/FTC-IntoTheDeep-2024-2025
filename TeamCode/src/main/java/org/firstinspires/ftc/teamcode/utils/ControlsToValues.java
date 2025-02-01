package org.firstinspires.ftc.teamcode.utils;

//Turns a joystick (or other) input into a speed value
//The point is that often, we want to have a non-linear relationship between joystick input and speed
//Ex. setting drive motor power to less than 0.2 usually results in the robot not moving
//so if the joystick input was 0.2, we would want a higher motor power
//Also, this helps with precision control, since more of the joystick input is used for lower, but nonzero speed
//Plug the equation into desmos to get a better understanding
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
