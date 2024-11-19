package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Slide;

@TeleOp
public class SlideTest extends OpMode {
//    Slide slideLeft;
    DcMotorEx slideRight;
    DcMotorEx slideLeft;
//    Slide slideRight;
    boolean moveSlideLeft = true;

    @Override
    public void init() {

        slideLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "liftRight"); slideRight.setDirection(DcMotorEx.Direction.REVERSE);
//        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));
//        slideRight = new Slide(hardwareMap.get(DcMotorEx.class, "liftRight"));
    }

    @Override
    public void loop() {
        slideLeft.setPower(-gamepad1.left_stick_y);
        slideRight.setPower(-gamepad1.left_stick_y);
    }
}
