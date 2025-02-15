package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.DualTurret;

@TeleOp
public class TestDualTurret extends OpMode {
    //    Turret turretLeft;
    DcMotorEx turretRight;
    DcMotorEx turretLeft;
    DualTurret turrets;
    //    Turret turretRight;
    boolean moveTurretLeft = true;
    boolean started = false;

    @Override
    public void init() {

        turretLeft = hardwareMap.get(DcMotorEx.class, "turretLeft");
        turretLeft.setDirection(DcMotorEx.Direction.REVERSE);
        turretRight = hardwareMap.get(DcMotorEx.class, "turretRight");
        turrets = new DualTurret(turretLeft, turretRight, true);
//        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "liftLeft"));
//        turretRight = new Turret(hardwareMap.get(DcMotorEx.class, "liftRight"));
    }

    @Override
    public void loop() {
        turrets.setAngleRadians(turrets.getAngleRadians()-gamepad1.left_stick_y*1);
        telemetry.addData("turret position: ", turrets.getEncoder());
        telemetry.update();
    }
}
