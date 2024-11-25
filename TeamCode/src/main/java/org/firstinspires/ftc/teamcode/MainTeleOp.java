package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Drive;
import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.Turret;

@TeleOp
public class MainTeleOp extends OpMode{
    Drive fod; Turret turretLeft; Slide slideLeft;
    DcMotorEx fl, fr, bl, br;
    Servo fingers, hand, wrist;
    final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    final double CLAW_GRAB_HEIGHT = 8.25;
    final double FINGER_CLOSE_POS = 0.5;
    final double FINGER_OPEN_POS = 0.0;
    final double HAND_START_POS = 0.25;
    final double HAND_PARALLEL_POS = 0.93; //Pos where hand is parallel with slides
    final double WRIST_START_POS = 0.43;
    final double WRIST_PERPEN_POS = 0.8; //Pos where wrist is perpendicular to slides
    boolean isGrabbing = false; boolean isOuttake = false; boolean isPreset = false;
    boolean xChanged = false; boolean bChanged = false;
    double targetGroundDistance = 8.0;
    double targetSlideLength = 10; double targetTurretAngle = Math.PI*45/180; double targetSlideVelocity = 0;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft"); fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft"); bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);
        fod = new Drive(fl, fr, bl, br, imu);
        fod.resetImu();

        turretLeft = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        slideLeft = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));

        fingers = hardwareMap.get(Servo.class, "fingers"); fingers.setPosition(FINGER_CLOSE_POS);
        hand = hardwareMap.get(Servo.class, "hand"); //0.93
        wrist = hardwareMap.get(Servo.class, "wrist"); wrist.setPosition(0.43);
    }

    @Override
    public void loop() {
        double[] motorPowers = fod.driveNormal(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("fl", motorPowers[0]);
        telemetry.addData("fr", motorPowers[1]);
        telemetry.addData("bl", motorPowers[2]);
        telemetry.addData("br", motorPowers[3]);

        if(gamepad2.x && xChanged == false) {
            if(!isGrabbing) {
                isGrabbing = true;
                targetGroundDistance = 8.0;
            }
            else {
                isGrabbing = false;
            }
            xChanged = true;
        }
        else if(!gamepad2.x){
            xChanged = false;
        }

        if(gamepad2.b && bChanged == false){
            if(fingers.getPosition() == FINGER_CLOSE_POS){
                fingers.setPosition(FINGER_OPEN_POS);
            }
            else{
                fingers.setPosition(FINGER_CLOSE_POS);
            }
            bChanged = true;
        }
        else if(!gamepad2.b){
            bChanged = false;
        }
        telemetry.addData("Fingers Position: ", fingers.getPosition());

        if(gamepad2.a){
            isGrabbing = false;
            isOuttake = false;
            targetSlideLength = 10;
            targetSlideVelocity = 1600;
            targetTurretAngle = Math.PI*45/180;
            fingers.setPosition(FINGER_CLOSE_POS);
            wrist.setPosition(WRIST_START_POS);
            hand.setPosition(HAND_START_POS);
            isPreset = true;
        }
        if(gamepad2.y){
            isGrabbing = false;
            isOuttake = false;
            targetSlideLength = 30;
            targetSlideVelocity = 1600;
            targetTurretAngle = Math.PI*160/180;
            fingers.setPosition(FINGER_CLOSE_POS);
            wrist.setPosition(WRIST_START_POS);
            hand.setPosition(HAND_START_POS);
            isPreset = true;
        }

        if(isGrabbing) {
//            targetGroundDistance -= gamepad2.left_stick_y*gamepad2.left_stick_y*0.6*Math.signum(gamepad2.left_stick_y);
//            targetGroundDistance = Math.max(targetGroundDistance, 6);
//            targetGroundDistance = Math.min(targetGroundDistance, 30);
//
//            turretLeft.setAngleRadians(Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT)), 1000000);
//            slideLeft.setLength(Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT, 2)),1000000);
//
//            targetTurretAngle = Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT));
//            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - CLAW_GRAB_HEIGHT, 2));

            hand.setPosition((Math.PI-(Math.PI/2-(Math.PI*10/180+targetTurretAngle)))*(HAND_PARALLEL_POS-HAND_START_POS)/(Math.PI*190/180)+HAND_START_POS);


            telemetry.addData("Turret Angle: ", turretLeft.getAngleDegrees());
            telemetry.addData("Slide Length: ", slideLeft.getLength());
            telemetry.addData("Target Ground Distance: ", targetGroundDistance);
        }
//        else {
            // TODO: Try switching motor modes here to reduce lag
            if(gamepad2.left_stick_y != 0) {
                isPreset = false;
                targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
                targetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI / 24, targetTurretAngle);
                targetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI / 24, targetTurretAngle);
            }

            if(gamepad2.right_stick_y != 0 || !isPreset) {
                isPreset = false;
                targetSlideLength = 10000 * -gamepad2.right_stick_y;
                targetSlideVelocity = gamepad2.right_stick_y * gamepad2.right_stick_y * 1600;
            }

            turretLeft.setAngleRadians(targetTurretAngle, 100000);
            slideLeft.setLength(targetSlideLength, targetSlideVelocity);
//        }

        if(gamepad2.dpad_right){
            wrist.setPosition(WRIST_START_POS);
        }
        else if(gamepad2.dpad_left){
            wrist.setPosition(WRIST_PERPEN_POS);
        }
        telemetry.addData("Wrist Position: ", wrist.getPosition());

        if(gamepad2.dpad_up){
            isOuttake = false;
            isGrabbing = false;
            hand.setPosition(hand.getPosition()-0.01);
        }
        else if((gamepad2.dpad_down || isOuttake)){
            isOuttake = true;
            isGrabbing = false;
            hand.setPosition(hand.getPosition()+0.01);
        }
        telemetry.addData("Hand Position: ", hand.getPosition());
        telemetry.update();
    }
}
