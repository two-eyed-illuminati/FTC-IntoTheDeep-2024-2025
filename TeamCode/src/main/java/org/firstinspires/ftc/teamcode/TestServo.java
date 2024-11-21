package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends OpMode {
    Servo handServo;
    Servo fingerServo;
    Servo wristServo;
    Servo servo;
    double position = 0.0;
    @Override
    public void init() {
        handServo = hardwareMap.get(Servo.class, "hand");
        fingerServo = hardwareMap.get(Servo.class, "fingers");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        servo = handServo;
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            servo = handServo;
        }
        if(gamepad1.b) {
            servo = fingerServo;
        }
        if(gamepad1.x) {
            servo = wristServo;
        }
        position += gamepad1.left_stick_y * 0.01;
        servo.setPosition(position);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.addData("position", position);
        telemetry.update();
    }
}
