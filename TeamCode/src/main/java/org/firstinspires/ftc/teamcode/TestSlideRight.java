package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Slide;

@TeleOp
public class TestSlideRight extends OpMode {
    DcMotorEx slideRight;
    Slide slide;

    @Override
    public void init() {
        slideRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        slideRight.setTargetPosition(0);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slide = new Slide(slideRight);
    }

    @Override
    public void loop() {
        double pulses = (5281.1 / 46.0) * (-gamepad1.left_stick_y*10000 - 10.0);
        pulses = Math.min(pulses, 2200);
        pulses = Math.max(pulses, 50);

        slideRight.setTargetPosition(200);
        slideRight.setVelocity(1000000);
        telemetry.addData("Encoder: ", slideRight.getCurrentPosition());
        telemetry.addData("Target: ", pulses);
//        slideRight.setTargetPosition((int)(-gamepad1.left_stick_y*10000));
//        slideRight.setVelocity(1000000);
//        slide.setLength(-gamepad1.left_stick_y*10000, 1000000);
    }
}
