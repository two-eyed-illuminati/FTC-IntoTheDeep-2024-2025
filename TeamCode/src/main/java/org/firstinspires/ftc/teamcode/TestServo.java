package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends OpMode {
    Servo servo;
    double position = 0.0;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "hand");
    }

    @Override
    public void loop() {
        position += gamepad1.left_stick_y * 0.01;
        servo.setPosition(position);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.addData("position", position);
        telemetry.update();
    }
}
