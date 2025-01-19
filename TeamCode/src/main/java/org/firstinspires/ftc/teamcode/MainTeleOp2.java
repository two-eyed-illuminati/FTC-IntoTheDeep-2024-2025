package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ControlsToValues;
import org.firstinspires.ftc.teamcode.utils.Drive;
import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.ToggleButton;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.List;

@TeleOp
public class MainTeleOp2 extends OpMode{
    Drive fod; Turret[] turrets; Slide[] slides;
    ControlsToValues slideCtv = new ControlsToValues();
    DcMotorEx fl, fr, bl, br;
    Servo[] fingers, hand, wrist;
    int currSide = 0; //0 = left, 1 = right
    final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    final double[] MAX_GROUND_DISTANCE = {10+13, 10+13}; //Max ground length of slides
    final double[] MAX_PRESET_GROUND_DISTANCE = {10+13/2.0, 10+13/2.0}; //have a lower limit for max slide length for presets
    final double[] MIN_GROUND_HEIGHT = {4, 4};
    final double[] FINGER_CLOSE_POS = {0.45, 0.19};
    final double[] FINGER_OPEN_POS = {0.8, 0.323};
    final double[] HAND_START_POS = {1.0, 0.6176};
    final double[] HAND_PARALLEL_POS = {0.4, 0.7249}; //Pos where hand is parallel with slides
    final double[] HAND_START_ANGLE = {Math.PI*0, Math.PI*0}; //Positive acute angle between hand start and slides
    final double[] WRIST_START_POS = {0.1, 1};
    final double[] WRIST_PERPEN_POS = {0.42, 0.6755}; //Pos where wrist is perpendicular to slides
    Gamepad2ControlState controlState = Gamepad2ControlState.PRESET;
    ToggleButton g1lT = new ToggleButton(); ToggleButton g2b = new ToggleButton(); ToggleButton g2rT = new ToggleButton();
    double targetGroundDistance = 12.0;
    double targetSlideLength = 10; double targetTurretAngle = Math.PI*45/180; double targetSlideVelocity = 0;
    double clawGrabHeight = 8.25;
    double targetHandAngle;
    List<LynxModule> allHubs;

    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI/2-turretAngle)+HAND_START_ANGLE[currSide])*(HAND_PARALLEL_POS[currSide]-HAND_START_POS[currSide])/(Math.PI+HAND_START_ANGLE[currSide])+HAND_START_POS[currSide];
    }
    public double angleFromHandPos(double pos, double turretAngle){
        return (pos-HAND_START_POS[currSide])*(Math.PI+HAND_START_ANGLE[currSide])/(HAND_PARALLEL_POS[currSide]-HAND_START_POS[currSide])+(Math.PI/2-turretAngle)-HAND_START_ANGLE[currSide];
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

        turrets = new Turret[2]; slides = new Slide[2];
        turrets[0] = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        slides[0] = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));
        slideCtv.cubicLowerSpeedValue = 0.2;

        fingers = new Servo[2]; hand = new Servo[2]; wrist = new Servo[2];
        fingers[0] = hardwareMap.get(Servo.class, "fingers"); fingers[0].setPosition(FINGER_CLOSE_POS[0]);
        hand[0] = hardwareMap.get(Servo.class, "hand"); hand[0].setPosition(HAND_START_POS[0]);
        wrist[0] = hardwareMap.get(Servo.class, "wrist"); wrist[0].setPosition(WRIST_START_POS[0]);

        turrets[1] = new Turret(hardwareMap.get(DcMotorEx.class, "turretRight"));
        slides[1] = new Slide(hardwareMap.get(DcMotorEx.class, "liftRight"));

        fingers[1] = hardwareMap.get(Servo.class, "speciFingers"); fingers[1].setPosition(FINGER_CLOSE_POS[1]);
        hand[1] = hardwareMap.get(Servo.class, "speciHand"); hand[1].setPosition(HAND_START_POS[1]);
        wrist[1] = hardwareMap.get(Servo.class, "speciWrist"); wrist[1].setPosition(WRIST_START_POS[1]);

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
        telemetry.addData("Turret Angle: ", turrets[currSide].getAngleDegrees());
        telemetry.addData("Slide Length: ", slides[currSide].getLength());

        //TODO: test
        double[] motorPowers = new double[10];
        if(controlState.equals(Gamepad2ControlState.GRAB)){
            //Drive a bit forward to make sure robot is as forward as it can be
            double yDrive = 0.3;

            //Try to get robot to align with submersible
            double angle = -fod.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Angle: ", angle);
            angle = angle % 2*Math.PI;
            angle = angle % (Math.PI/2);
            angle -= Math.PI/4;
            telemetry.addData("Angle: ", angle);

            double rotation = -angle;
            rotation = Math.min(0.3, Math.max(-0.3, rotation));

            motorPowers = fod.driveNormal(gamepad1.left_stick_x*0.5, yDrive, rotation);

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

        if(g2rT.activated(gamepad2.right_trigger > 0.8)) {
            currSide = 1 - currSide;
        }

        if(g1lT.activated(gamepad1.left_trigger > 0.8)) {
            if(!controlState.equals(Gamepad2ControlState.GRAB)) {
                controlState = Gamepad2ControlState.GRAB;
                targetGroundDistance = 12.0;
                clawGrabHeight = 8.25;
                fingers[currSide].setPosition(FINGER_OPEN_POS[currSide]);
            }
            else {
                controlState = Gamepad2ControlState.MANUAL_CONTROL;
            }
        }

        if(g2b.activated(gamepad2.b)) {
            if(Math.abs(fingers[currSide].getPosition() - FINGER_CLOSE_POS[currSide]) < 0.1){
                fingers[currSide].setPosition(FINGER_OPEN_POS[currSide]);
            }
            else{
                fingers[currSide].setPosition(FINGER_CLOSE_POS[currSide]);
            }
        }

        if(gamepad2.a){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = 10;
            targetSlideVelocity = (537.7*312.0/60.0);
            targetTurretAngle = Math.PI*50/180;
            fingers[currSide].setPosition(FINGER_CLOSE_POS[currSide]);
            wrist[currSide].setPosition(WRIST_START_POS[currSide]);
            hand[currSide].setPosition(HAND_START_POS[currSide]);
        }
        if(gamepad2.x){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = Math.sqrt(Math.pow(11.5, 2) + Math.pow(12, 2));
            targetSlideVelocity = (537.7*312.0/60.0);
            targetTurretAngle = Math.atan2(11.5, 12)+Math.PI*90/180;
            fingers[currSide].setPosition(FINGER_CLOSE_POS[currSide]);
            wrist[currSide].setPosition(WRIST_PERPEN_POS[currSide]);
            hand[currSide].setPosition(handPosFromAngle(Math.PI*180/180, Math.atan2(11.5, 12)+Math.PI*90/180));
        }
        if(gamepad2.y){
            controlState = Gamepad2ControlState.PRESET;
            targetSlideLength = 40;
            targetSlideVelocity = (537.7*312.0/60.0);
            targetTurretAngle = Math.PI*180/180;
            fingers[currSide].setPosition(FINGER_CLOSE_POS[currSide]);
            wrist[currSide].setPosition(WRIST_START_POS[currSide]);
            hand[currSide].setPosition(handPosFromAngle(Math.PI*90/180, Math.PI*180/180));
        }

        if(gamepad2.dpad_right){
            wrist[currSide].setPosition(WRIST_START_POS[currSide]);
        }
        else if(gamepad2.dpad_left){
            wrist[currSide].setPosition(WRIST_PERPEN_POS[currSide]);
        }

        if(gamepad2.right_bumper) {
            controlState = Gamepad2ControlState.MANUAL_CONTROL;
        }

        if(controlState.equals(Gamepad2ControlState.GRAB)) {
            targetGroundDistance -= gamepad1.left_stick_y*gamepad1.left_stick_y*0.6*Math.signum(gamepad1.left_stick_y);
            targetGroundDistance = Math.max(targetGroundDistance, 6.0);
            targetGroundDistance = Math.min(targetGroundDistance, MAX_GROUND_DISTANCE[currSide]);
            clawGrabHeight += gamepad1.right_stick_y*gamepad1.right_stick_y*0.3*Math.signum(gamepad1.right_stick_y);
            clawGrabHeight = Math.max(clawGrabHeight, MIN_GROUND_HEIGHT[currSide]);

            turrets[currSide].setAngleRadians(Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight)));
            slides[currSide].setLength(Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2)));

            targetTurretAngle = Math.atan(targetGroundDistance / (SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight));
            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

            hand[currSide].setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));
        }
        if(controlState.equals(Gamepad2ControlState.PRESET)){
            if(gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0){
                controlState = Gamepad2ControlState.MANUAL_CONTROL;
            }
            else {
                double currTargetSlideLength = targetSlideLength;
                double currTargetTurretAngle = targetTurretAngle;
                double currTargetSlideVelocity = targetSlideVelocity;
                double futureGroundDistance = Math.sin(turrets[currSide].getAngleRadians()) * slides[currSide].getLength();
                if(futureGroundDistance > MAX_PRESET_GROUND_DISTANCE[currSide]){
                    currTargetSlideLength = MAX_PRESET_GROUND_DISTANCE[currSide] / Math.sin(turrets[currSide].getAngleRadians());
                    currTargetSlideVelocity = (537.7*312.0/60.0);
                }
                double futureGroundHeight = SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turrets[currSide].getAngleRadians()) * slides[currSide].getLength();
                if(futureGroundHeight < MIN_GROUND_HEIGHT[currSide]){
                    currTargetTurretAngle = Math.acos((SLIDE_PIVOT_GROUND_HEIGHT - MIN_GROUND_HEIGHT[currSide]) / slides[currSide].getLength());
                    currTargetTurretAngle = Math.min(turrets[currSide].getAngleRadians() + Math.PI / 12, currTargetTurretAngle);
                    currTargetTurretAngle = Math.max(turrets[currSide].getAngleRadians() - Math.PI / 12, currTargetTurretAngle);
                }
                turrets[currSide].setAngleRadians(currTargetTurretAngle);
                slides[currSide].setLength(currTargetSlideLength, currTargetSlideVelocity);
            }
        }
        if(controlState.equals(Gamepad2ControlState.MANUAL_CONTROL)){
            targetSlideLength = 10000 * -gamepad2.right_stick_y;
            targetSlideVelocity = slideCtv.targetSpeedFromJoysticks(-gamepad2.right_stick_y) * (537.7*312.0/60.0);

            targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
            targetTurretAngle = Math.min(turrets[currSide].getAngleRadians() + Math.PI / 24, targetTurretAngle);
            targetTurretAngle = Math.max(turrets[currSide].getAngleRadians() - Math.PI / 24, targetTurretAngle);

            double futureGroundDistance = Math.sin(targetTurretAngle) * slides[currSide].getLength();
            if(futureGroundDistance > MAX_GROUND_DISTANCE[currSide]){
                targetSlideLength = MAX_GROUND_DISTANCE[currSide] / Math.sin(targetTurretAngle);
                targetSlideVelocity = (537.7*312.0/60.0);
            }
            double futureGroundHeight = SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(targetTurretAngle) * slides[currSide].getLength();
            if(futureGroundHeight < MIN_GROUND_HEIGHT[currSide]){
                targetTurretAngle = Math.acos((SLIDE_PIVOT_GROUND_HEIGHT - MIN_GROUND_HEIGHT[currSide]) / slides[currSide].getLength());
                targetTurretAngle = Math.min(turrets[currSide].getAngleRadians() + Math.PI / 12, targetTurretAngle);
                targetTurretAngle = Math.max(turrets[currSide].getAngleRadians() - Math.PI / 12, targetTurretAngle);
            }

            turrets[currSide].setAngleRadians(targetTurretAngle);
            slides[currSide].setLength(targetSlideLength, targetSlideVelocity);
        }

        telemetry.update();
    }
}