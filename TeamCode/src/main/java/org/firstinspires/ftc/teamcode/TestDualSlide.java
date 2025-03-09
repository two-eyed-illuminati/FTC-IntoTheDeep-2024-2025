package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.DualSlide;

@TeleOp
public class TestDualSlide extends OpMode {
    //    Slide slideLeft;
    DcMotorEx slideRight;
    DcMotorEx slideLeft;
    DualSlide slides;
    //    Slide slideRight;
    boolean moveSlideLeft = true;
    boolean started = false;

    @Override
    public void init() {

        slideLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slideRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        slides = new DualSlide(slideLeft, slideRight, true);
//        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));
//        slideRight = new Slide(hardwareMap.get(DcMotorEx.class, "liftRight"));
    }

    @Override
    public void loop() {
        slides.setLength(slides.getLength()-gamepad1.left_stick_y*1);
        telemetry.addData("slide position: ", slides.getEncoder()[0]);
        telemetry.addData("slide2 pos: ", slides.getEncoder()[1]);
        telemetry.update();
    }
}
