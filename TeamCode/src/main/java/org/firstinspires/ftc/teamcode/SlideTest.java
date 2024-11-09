package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Slide;

@TeleOp
public class SlideTest extends OpMode {
    Slide slideLeft;
    Slide slideRight;
    boolean moveSlideLeft = true;

    @Override
    public void init() {
        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "slideLeft"));
        slideRight = new Slide(hardwareMap.get(DcMotorEx.class, "slideRight"));

    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            moveSlideLeft = true;
        }
        else if(gamepad1.b) {
            moveSlideLeft = false;
        }
        Slide slide = moveSlideLeft ? slideLeft : slideRight;
        slide.setLength(slide.getLength()+(-gamepad1.left_stick_y*10000), Math.abs(gamepad1.left_stick_y*100));
        telemetry.addData("Slide Length: ", slide.getLength());
        telemetry.update();
    }
}
