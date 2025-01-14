package org.firstinspires.ftc.teamcode.utils;

public class PID {
    public double kP, kI, kD;
    public double integral, lastError;

    public PID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        integral = 0;
        lastError = 0;
    }

    public double getOutput(double error){
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset(){
        integral = 0;
        lastError = 0;
    }
}
