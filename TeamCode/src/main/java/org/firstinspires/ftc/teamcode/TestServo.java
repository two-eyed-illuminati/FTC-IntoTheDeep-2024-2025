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
    boolean started = false;
    @Override
    public void init() {
        handServo = hardwareMap.get(Servo.class, "hand");
        fingerServo = hardwareMap.get(Servo.class, "fingers");
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            servo = handServo;
            position = servo.getPosition();
            started = true;
        }
        if(gamepad1.b) {
            servo = fingerServo;
            position = servo.getPosition();
            started = true;
        }
        if(gamepad1.x) {
            servo = wristServo;
            position = servo.getPosition();
            started = true;
        }
        position -= gamepad1.left_stick_y * 0.003 * (gamepad1.left_trigger > 0.8 ? 0.1 : 1.0);
        position = Math.min(1.0, Math.max(0.0, position));
        telemetry.addLine("A: hand, B: fingers, X: wrist");
        if(started){
            servo.setPosition(position);
            telemetry.addData("servo position", servo.getPosition());
            telemetry.addData("position", position);
        }
        telemetry.update();
    }
}
