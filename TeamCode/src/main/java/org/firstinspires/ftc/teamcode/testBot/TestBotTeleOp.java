package org.firstinspires.ftc.teamcode.testBot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.teamcode.utils.FieldOrientedDrive;

@TeleOp
public class TestBotTeleOp extends OpMode {
    FieldOrientedDrive fod;

    @Override
    public void init() {
        DcMotor fl = hardwareMap.dcMotor.get("frontLeft"); fl.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor fr = hardwareMap.dcMotor.get("frontRight");
        DcMotor bl = hardwareMap.dcMotor.get("backLeft"); bl.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor br = hardwareMap.dcMotor.get("backRight");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        IMU imu = hardwareMap.get(IMU.class, "imu"); imu.initialize(imuParameters);
        fod = new FieldOrientedDrive(fl, fr, bl, br, hardwareMap.get(IMU.class, "imu"));
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
    }
}
