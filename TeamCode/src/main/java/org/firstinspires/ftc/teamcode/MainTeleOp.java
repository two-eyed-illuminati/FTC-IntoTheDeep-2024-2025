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
    final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    final double CLAW_GRAB_HEIGHT = 6.5;
    boolean isGrabbing = false;
    FieldOrientedDrive fod;
    Turret turretLeft;
    Slide slideLeft;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    double targetGroundDistance = 12.0;
    double targetSlideLength = 10;
    double targetTurretAngle = Math.PI/4;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft"); fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft"); bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);
        fod = new FieldOrientedDrive(fl, fr, bl, br, imu);
        fod.resetImu();

        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));
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
            targetGroundDistance = 8.0;
        }
        else if(gamepad2.b) {
            isGrabbing = false;
        }

        if(isGrabbing) {
            targetGroundDistance -= gamepad2.left_stick_y*0.4;
            targetGroundDistance = Math.max(targetGroundDistance, 6);
            targetGroundDistance = Math.min(targetGroundDistance, 30);

            turretLeft.setAngleRadians(Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT)), Math.abs(gamepad2.left_stick_y * 50000000));
            slideLeft.setLength(Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT, 2)), Math.abs(gamepad2.left_stick_y * 5000000));

            targetTurretAngle = Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT));
            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT, 2));

            telemetry.addData("Turret Angle: ", turretLeft.getAngleDegrees());
            telemetry.addData("Slide Length: ", slideLeft.getLength());
            telemetry.addData("Target Ground Distance: ", targetGroundDistance);

            telemetry.update();
        }
        else {
            targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
            targetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI/24, targetTurretAngle);
            targetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI/24, targetTurretAngle);
            targetSlideLength -= gamepad2.right_stick_y * gamepad2.right_stick_y * 1 * Math.signum(gamepad2.right_stick_y);
            turretLeft.setAngleRadians(targetTurretAngle, 100000);
            slideLeft.setLength(targetSlideLength, 1000000);
        }
    }
}
