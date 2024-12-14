package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class MoveLeftAuto extends LinearOpMode {
    DcMotorEx fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();

        fl.setPower(0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(0.5);
        sleep(300);

        fl.setPower(-0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(-0.5);
        sleep(12000);
    }
}
