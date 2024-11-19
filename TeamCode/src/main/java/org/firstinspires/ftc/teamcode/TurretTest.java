package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Turret;

@TeleOp
public class TurretTest extends OpMode {
    Turret turretLeft;
    Turret turretRight;
    DcMotorEx turretLeftMotor;
    DcMotorEx turretRightMotor;
    boolean moveTurretLeft = true;

    @Override
    public void init() {
//        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        turretLeftMotor = hardwareMap.get(DcMotorEx.class, "turretLeft");
        turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turretRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        turretRight = new Turret(turretRightMotor);
    }

    @Override
    public void loop() {
        turretLeftMotor.setPower(gamepad1.left_stick_y);
        turretRightMotor.setPower(gamepad1.right_stick_y);
    }
}
