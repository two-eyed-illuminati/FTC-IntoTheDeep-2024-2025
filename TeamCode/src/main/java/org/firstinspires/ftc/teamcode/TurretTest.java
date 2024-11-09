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
    boolean moveTurretLeft = true;

    @Override
    public void init() {
        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        turretRight = new Turret(hardwareMap.get(DcMotorEx.class, "turretRight"));
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            moveTurretLeft = true;
        }
        else if(gamepad1.b) {
            moveTurretLeft = false;
        }
        Turret turret = moveTurretLeft ? turretLeft : turretRight;
        turret.setAngleRadians(turret.getAngleRadians()+(-(-gamepad1.left_stick_y*10000)), Math.abs(gamepad1.left_stick_y*500));
    }
}
