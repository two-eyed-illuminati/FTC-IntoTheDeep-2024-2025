package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.utils.Turret;

@TeleOp
public class MainTeleOp extends OpMode{
    FieldOrientedDrive fod;
    Turret turret;
    double maxVelocity = 0;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx liftMotor;

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

        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretLeft");
        turret = new Turret(turretMotor);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft"); liftMotor.setTargetPosition(0); liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        maxVelocity += gamepad2.right_stick_y*10;
        double[] motorPowers = fod.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("fl", motorPowers[0]);
        telemetry.addData("fr", motorPowers[1]);
        telemetry.addData("bl", motorPowers[2]);
        telemetry.addData("br", motorPowers[3]);
        telemetry.addData("targetFieldOrientedMoveHeading", motorPowers[4]);
        telemetry.addData("currRobotHeading", motorPowers[9]);


        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + (int)(-gamepad2.left_stick_y*maxVelocity));
        telemetry.addData("Lift Encoder Value: ", liftMotor.getCurrentPosition());
//        turret.setAngleRadians(-gamepad2.left_stick_y*0.1+turret.getAngleRadians(), maxVelocity);
//        telemetry.addData("Target Turret Angle (Deg): ", -gamepad2.left_stick_y*0.1+turret.getAngleDegrees());
//        telemetry.addData("Current Turret Angle (Deg): ", turret.getAngleDegrees());
//        telemetry.addData("Target Encoder Value: ", turret.getAngleDegrees()*5281.1*4/360);
//        telemetry.addData("Current Encoder Value: ", turret.getAngleDegrees()*5281.1*4/360);
//        telemetry.addData("Max Turret Speed: ", maxVelocity);
        telemetry.update();
    }
}
