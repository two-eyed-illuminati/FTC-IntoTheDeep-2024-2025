package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ControlsToValues;
import org.firstinspires.ftc.teamcode.utils.Drive;
import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.ToggleButton;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.List;

enum Gamepad2ControlState{
    GRAB, MANUAL_CONTROL, PRESET, OUTTAKE
}

@TeleOp
public class MainTeleOp extends OpMode{
    Drive fod; Turret turretLeft; Slide slideLeft;
    Turret turretRight; Slide slideRight;
    Servo speciFingers, speciHand, speciWrist;
    ControlsToValues slideCtv = new ControlsToValues();
    DcMotorEx fl, fr, bl, br;
    Servo fingers, hand, wrist;
    final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    final double MAX_GROUND_DISTANCE = 10+13; //Max ground length of slides
    final double MAX_PRESET_GROUND_DISTANCE = 10+13/2.0; //have a lower limit for max slide length for presets
    final double MIN_GROUND_HEIGHT = 5;
    final double FINGER_CLOSE_POS = 0.309-0.035;
    final double FINGER_OPEN_POS = 0.386-0.035;
    final double HAND_START_POS = 1.0;
    final double HAND_PARALLEL_POS = 0.4; //Pos where hand is parallel with slides
    final double HAND_START_ANGLE = Math.PI*0; //Positive acute angle between hand start and slides
    final double WRIST_START_POS = 0.1341;
    final double WRIST_PERPEN_POS = 0.32+WRIST_START_POS; //Pos where wrist is perpendicular to slides
    Gamepad2ControlState controlState = Gamepad2ControlState.PRESET;
    ToggleButton g1lT = new ToggleButton(); ToggleButton g2b = new ToggleButton(); ToggleButton g1a = new ToggleButton();
    double targetGroundDistance = 18.0;
    double targetSlideLength = 10; double targetTurretAngle = Math.PI*45/180; double targetSlideVelocity = 0;
    double clawGrabHeight = 11;
    double targetHandAngle;
    double targetHandPresetPos;
    double targetHandStartMoveTurretPos;
    List<LynxModule> allHubs;

    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI/2-turretAngle)+HAND_START_ANGLE)*(HAND_PARALLEL_POS-HAND_START_POS)/(Math.PI+HAND_START_ANGLE)+HAND_START_POS;
    }
    public double angleFromHandPos(double pos, double turretAngle){
        return (pos-HAND_START_POS)*(Math.PI+HAND_START_ANGLE)/(HAND_PARALLEL_POS-HAND_START_POS)+(Math.PI/2-turretAngle)-HAND_START_ANGLE;
    }
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
        slideCtv.cubicLowerSpeedValue = 0.2;

        fingers = hardwareMap.get(Servo.class, "fingers"); fingers.setPosition(FINGER_CLOSE_POS);
        hand = hardwareMap.get(Servo.class, "hand"); hand.setPosition(HAND_START_POS);
        targetHandPresetPos = HAND_START_POS;
        targetHandStartMoveTurretPos = Math.PI*0/180;
        wrist = hardwareMap.get(Servo.class, "wrist"); wrist.setPosition(WRIST_START_POS);

        DcMotorEx turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turretRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretRight = new Turret(turretRightMotor);
        DcMotorEx slideRightMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        slideRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight = new Slide(slideRightMotor);
        slideRight.setLength(10);
        turretRight.setAngleRadians(Math.PI*45/180);

        speciFingers = hardwareMap.get(Servo.class, "speciFingers");
        speciHand = hardwareMap.get(Servo.class, "speciHand");
        speciWrist = hardwareMap.get(Servo.class, "speciWrist");
        speciFingers.setPosition(0.19);
        speciHand.setPosition(0.6176+0.015);
        speciWrist.setPosition(1);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        telemetry.addData("MODE: ", controlState.name());
        telemetry.addData("Turret Angle: ", turretLeft.getAngleDegrees());
        telemetry.addData("Slide Length: ", slideLeft.getLength());
        telemetry.addData("Hand Pos: ", hand.getPosition());
        telemetry.addData("Wrist Pos: ", wrist.getPosition());
        telemetry.addData("Fingers Pos: ", fingers.getPosition());

        //TODO: test
        double[] motorPowers = new double[10];
        if(controlState.equals(Gamepad2ControlState.GRAB)){
            motorPowers = fod.driveFieldCentric(gamepad1.left_stick_x*0.5, gamepad1.left_stick_y*-0.5, gamepad1.right_stick_x*0.5);

        }
        else{
            double xDrive = gamepad1.left_stick_x * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
            double yDrive = -gamepad1.left_stick_y * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
            double rotation = gamepad1.right_stick_x * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
            motorPowers = fod.driveFieldCentric(xDrive, yDrive, rotation);
        }
        telemetry.addData("fl", motorPowers[0]);
        telemetry.addData("fr", motorPowers[1]);
        telemetry.addData("bl", motorPowers[2]);
        telemetry.addData("br", motorPowers[3]);

        if(g1lT.activated(gamepad1.left_trigger > 0.8)) {
            if(!controlState.equals(Gamepad2ControlState.GRAB)) {
                controlState = Gamepad2ControlState.GRAB;
                targetGroundDistance = 18.0;
                clawGrabHeight = 11;
                fingers.setPosition(FINGER_OPEN_POS);
                wrist.setPosition(WRIST_PERPEN_POS);
            }
            else {
                controlState = Gamepad2ControlState.PRESET;
                targetSlideLength = 10;
                targetSlideVelocity = (537.7*312.0/60.0);
                targetTurretAngle = Math.PI*50/180;
                fingers.setPosition(FINGER_CLOSE_POS);
                wrist.setPosition(WRIST_START_POS);
                targetHandPresetPos = HAND_START_POS;
                targetHandStartMoveTurretPos = Math.PI*0/180;
            }
        }

        if(g2b.activated(gamepad2.b) || (controlState.equals(Gamepad2ControlState.GRAB) && g1a.activated(gamepad1.b))) {
            if(Math.abs(fingers.getPosition() - FINGER_CLOSE_POS) < 0.02){
                fingers.setPosition(FINGER_OPEN_POS);
            }
            else{
                fingers.setPosition(FINGER_CLOSE_POS);
            }
        }

        if(gamepad2.a){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = 10;
            targetSlideVelocity = (537.7*312.0/60.0);
            targetTurretAngle = Math.PI*50/180;
            fingers.setPosition(FINGER_CLOSE_POS);
            wrist.setPosition(WRIST_START_POS);
            targetHandPresetPos = HAND_START_POS;
            targetHandStartMoveTurretPos = Math.PI*0/180;
        }
        if(gamepad2.x){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = Math.sqrt(Math.pow(11.5, 2) + Math.pow(12, 2));
            targetSlideVelocity = (537.7*312.0/60.0);
            targetTurretAngle = Math.atan2(11.5, 12)+Math.PI*90/180;
            fingers.setPosition(FINGER_CLOSE_POS);
            wrist.setPosition(WRIST_PERPEN_POS);
            targetHandPresetPos = handPosFromAngle(Math.PI*180/180, Math.atan2(11.5, 12)+Math.PI*90/180);
            targetHandStartMoveTurretPos = Math.PI*90/180;
        }
        if(gamepad2.y){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = 40;
            targetSlideVelocity = 0.75*(537.7*312.0/60.0);
            targetTurretAngle = Math.PI*165/180;
            fingers.setPosition(FINGER_CLOSE_POS);
            wrist.setPosition(WRIST_START_POS);
            targetHandPresetPos = handPosFromAngle(Math.PI*180/180, Math.PI*165/180);
            targetHandStartMoveTurretPos = Math.PI*90/180;
        }

        if(gamepad2.dpad_right){
            wrist.setPosition(WRIST_START_POS);
        }
        else if(gamepad2.dpad_left){
            wrist.setPosition(WRIST_PERPEN_POS);
        }
        if(controlState.equals(Gamepad2ControlState.GRAB) && gamepad1.dpad_right){
            if(wrist.getPosition() > WRIST_START_POS+0.02){
                wrist.setPosition(wrist.getPosition()-(WRIST_PERPEN_POS-WRIST_START_POS)/2.0);
            }
        }
        else if(controlState.equals(Gamepad2ControlState.GRAB) && gamepad1.dpad_left){
            if(wrist.getPosition() < WRIST_PERPEN_POS-0.02){
                wrist.setPosition(wrist.getPosition()+(WRIST_PERPEN_POS-WRIST_START_POS)/2.0);
            }
        }

        if(gamepad2.dpad_up){
            controlState = Gamepad2ControlState.OUTTAKE;
            targetHandAngle = Math.PI*180/180;
        }
//        else if(gamepad2.dpad_down){
//            controlState = Gamepad2ControlState.OUTTAKE;
//            targetHandAngle = Math.PI*0/180;
//        }

        if(gamepad2.right_bumper) {
            controlState = Gamepad2ControlState.MANUAL_CONTROL;
        }

        if(controlState.equals(Gamepad2ControlState.GRAB)) {
//            targetGroundDistance -= gamepad1.left_stick_y*gamepad1.left_stick_y*0.6*Math.signum(gamepad1.left_stick_y);
            if(gamepad1.right_trigger > 0.8) {
                targetGroundDistance += gamepad1.dpad_up ? 0.3 : 0;
                targetGroundDistance += gamepad1.dpad_down ? -0.3 : 0;
            }
            else{
                targetGroundDistance += gamepad1.dpad_up ? 0.6 : 0;
                targetGroundDistance += gamepad1.dpad_down ? -0.6 : 0;
            }
            targetGroundDistance = Math.max(targetGroundDistance, 6.0);
            targetGroundDistance = Math.min(targetGroundDistance, MAX_GROUND_DISTANCE);
            clawGrabHeight += gamepad1.y ? (gamepad1.right_trigger > 0.8 ? 0.5 : 1) : 0;
            clawGrabHeight -= gamepad1.a ? (gamepad1.right_trigger > 0.8 ? 0.5 : 1) : 0;
            clawGrabHeight = Math.max(clawGrabHeight, MIN_GROUND_HEIGHT);
            clawGrabHeight = Math.min(clawGrabHeight, SLIDE_PIVOT_GROUND_HEIGHT-0.01);

            turretLeft.setAngleRadians(Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight)));
            slideLeft.setLength(Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2)));

            targetTurretAngle = Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight));
            targetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI / 12, targetTurretAngle);
            targetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI / 12, targetTurretAngle);
            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

            telemetry.addData("Target Turret Angle: ", targetTurretAngle);
            telemetry.addData("Target Hand Pos: ", handPosFromAngle(Math.PI*270/180, targetTurretAngle));
            hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));
        }
        if(controlState.equals(Gamepad2ControlState.PRESET)){
            if(gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0){
                controlState = Gamepad2ControlState.MANUAL_CONTROL;
            }
            else {
                double currTargetSlideLength = targetSlideLength;
                double currTargetTurretAngle = targetTurretAngle;
                double currTargetSlideVelocity = targetSlideVelocity;
                double futureGroundDistance = Math.sin(turretLeft.getAngleRadians()) * slideLeft.getLength();
                if(futureGroundDistance > MAX_PRESET_GROUND_DISTANCE){
                    currTargetSlideLength = MAX_PRESET_GROUND_DISTANCE / Math.sin(turretLeft.getAngleRadians());
                }
                if(Math.abs(futureGroundDistance - MAX_PRESET_GROUND_DISTANCE) < 2){
                    currTargetSlideVelocity = 0.1*targetSlideVelocity;
                }
                if(turretLeft.getAngleRadians() > targetHandStartMoveTurretPos){
                    hand.setPosition(targetHandPresetPos);
                }
//                double futureGroundHeight = SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turretLeft.getAngleRadians()) * slideLeft.getLength();
//                if(futureGroundHeight < MIN_GROUND_HEIGHT){
//                    currTargetTurretAngle = Math.acos((SLIDE_PIVOT_GROUND_HEIGHT - MIN_GROUND_HEIGHT) / slideLeft.getLength());
//                    currTargetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI / 12, currTargetTurretAngle);
//                    currTargetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI / 12, currTargetTurretAngle);
//                }
                turretLeft.setAngleRadians(currTargetTurretAngle);
                slideLeft.setLength(currTargetSlideLength, currTargetSlideVelocity);
            }
        }
        if(controlState.equals(Gamepad2ControlState.MANUAL_CONTROL) || controlState.equals(Gamepad2ControlState.OUTTAKE)){
            targetSlideLength = 10000 * -gamepad2.right_stick_y;
            targetSlideVelocity = slideCtv.targetSpeedFromJoysticks(-gamepad2.right_stick_y) * (537.7*312.0/60.0);

            targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
            targetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI / 24, targetTurretAngle);
            targetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI / 24, targetTurretAngle);

            double futureGroundDistance = Math.sin(targetTurretAngle) * slideLeft.getLength();
            if(futureGroundDistance > MAX_GROUND_DISTANCE){
                targetSlideLength = MAX_GROUND_DISTANCE / Math.sin(targetTurretAngle);
                targetSlideVelocity = (537.7*312.0/60.0);
            }
            double futureGroundHeight = SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(targetTurretAngle) * slideLeft.getLength();
            if(futureGroundHeight < MIN_GROUND_HEIGHT){
                targetTurretAngle = Math.acos((SLIDE_PIVOT_GROUND_HEIGHT - MIN_GROUND_HEIGHT) / slideLeft.getLength());
                targetTurretAngle = Math.min(turretLeft.getAngleRadians() + Math.PI / 12, targetTurretAngle);
                targetTurretAngle = Math.max(turretLeft.getAngleRadians() - Math.PI / 12, targetTurretAngle);
            }

            turretLeft.setAngleRadians(targetTurretAngle);
            slideLeft.setLength(targetSlideLength, targetSlideVelocity);
        }
        if(controlState.equals(Gamepad2ControlState.OUTTAKE)){
            hand.setPosition(handPosFromAngle(targetHandAngle, turretLeft.getAngleRadians()));
        }

        telemetry.update();
    }
}