package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.Turret;

@TeleOp
public class MainTeleOp extends OpMode{
    final double SLIDE_PIVOT_GROUND_HEIGHT = -1.0; //TODO figure out later
    final double CLAW_GRAB_HEIGHT = 5.0;
    boolean isGrabbing = false;
    FieldOrientedDrive fod;
    Turret turretLeft;
    Slide slideLeft;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    @Override
    public void init() {
        //TODO Remember to reverse motors if necessary
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft"); fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight"); fr.setDirection(DcMotorEx.Direction.REVERSE);
        bl = hardwareMap.get(DcMotorEx.class, "backLeft"); bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //TODO Remember to Orient IMU correctly
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);
        fod = new FieldOrientedDrive(fl, fr, bl, br, imu);
        fod.resetImu();

        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "slideLeft"));
    }

    @Override
    public void loop() {
        double[] motorPowers = fod.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("fl", motorPowers[0]);
        telemetry.addData("fr", motorPowers[1]);
        telemetry.addData("bl", motorPowers[2]);
        telemetry.addData("br", motorPowers[3]);
        telemetry.addData("targetFieldOrientedMoveHeading", motorPowers[4]);
        telemetry.addData("currRobotHeading", motorPowers[9]);

        if(gamepad2.a) {
            isGrabbing = true;
        }
        else if(gamepad2.b) {
            isGrabbing = false;
        }

        if(isGrabbing) {
            double targetGroundDistance = Math.tan(turretLeft.getAngleRadians()) * (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT) - gamepad2.left_stick_y;

            turretLeft.setAngleRadians(Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT)), Math.abs(gamepad2.left_stick_y * 500));
            slideLeft.setLength(Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT, 2)), Math.abs(gamepad2.left_stick_y * 500));

            telemetry.update();
        }
    }
}
