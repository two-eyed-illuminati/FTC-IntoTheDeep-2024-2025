package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ControlsToValues;
import org.firstinspires.ftc.teamcode.utils.Drive;
import org.firstinspires.ftc.teamcode.utils.DualSlide;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualTurret;
import org.firstinspires.ftc.teamcode.utils.DualTurretAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.ToggleButton;

import java.util.List;

enum Gamepad2ControlState{
    GRAB, MANUAL_CONTROL, PRESET
}

@TeleOp
public class MainTeleOp extends OpMode{
    Drive fod;
    DualTurret turrets; DualSlide slides;
    ControlsToValues slideCtv = new ControlsToValues();
    DcMotorEx fl, fr, bl, br;
    Servo fingers, hand, wrist;
    Gamepad2ControlState controlState = Gamepad2ControlState.PRESET;
    ToggleButton g1lT = new ToggleButton(); ToggleButton g2b = new ToggleButton(); ToggleButton g1b = new ToggleButton();
    ToggleButton g1dR = new ToggleButton(); ToggleButton g1dL = new ToggleButton();
    double targetGroundDistance = 18.0; double clawGrabHeight = 11;
    double targetSlideLength = 10; double targetSlideVelocity = 0;
    double targetTurretAngle = Math.PI*60/180;
    Action presetAction = new NullAction();
    List<LynxModule> allHubs;

    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI/2-turretAngle)+RobotConstants.HAND_START_ANGLE)*(RobotConstants.HAND_PARALLEL_POS-RobotConstants.HAND_START_POS)/(Math.PI+RobotConstants.HAND_START_ANGLE)+RobotConstants.HAND_START_POS;
    }
    public double angleFromHandPos(double pos, double turretAngle){
        return (pos-RobotConstants.HAND_START_POS)*(Math.PI+RobotConstants.HAND_START_ANGLE)/(RobotConstants.HAND_PARALLEL_POS-RobotConstants.HAND_START_POS)+(Math.PI/2-turretAngle)-RobotConstants.HAND_START_ANGLE;
    }
    @Override
    public void init() {
        telemetry.addLine("REMINDER: Lift turret up and retract slides");
        telemetry.update();

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);
        fod = new Drive(fl, fr, bl, br, imu);
        fod.resetImu();

        DcMotorEx turretLeftMotor = hardwareMap.get(DcMotorEx.class, "turretLeft"); turretLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turrets = new DualTurret(turretLeftMotor, turretRightMotor);

        DcMotorEx slideLeftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft"); slideLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx slideRightMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        slides = new DualSlide(slideLeftMotor, slideRightMotor);

        slideCtv.cubicLowerSpeedValue = 0.2;

        fingers = hardwareMap.get(Servo.class, "fingers"); fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
        hand = hardwareMap.get(Servo.class, "hand"); hand.setPosition(RobotConstants.HAND_START_POS);
        wrist = hardwareMap.get(Servo.class, "wrist"); wrist.setPosition(RobotConstants.WRIST_START_POS);

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
        telemetry.addData("Turret Angle: ", turrets.getAngleDegrees());
        telemetry.addData("Slide Length: ", slides.getLength());
        telemetry.addData("Hand Pos: ", hand.getPosition());
        telemetry.addData("Wrist Pos: ", wrist.getPosition());
        telemetry.addData("Fingers Pos: ", fingers.getPosition());

        //Switching Control Modes
        if(g1lT.activated(gamepad1.left_trigger > 0.8)) {
            if(!controlState.equals(Gamepad2ControlState.GRAB)) {
                controlState = Gamepad2ControlState.GRAB;
                targetGroundDistance = 18.0;
                clawGrabHeight = 11;
                fingers.setPosition(RobotConstants.FINGER_OPEN_POS);
                wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
            }
            else {
                controlState = Gamepad2ControlState.PRESET;
                presetAction = new ParallelAction(
                        new DualSlideSetLength(slides, 10),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*60/180)
                );
                fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                wrist.setPosition(RobotConstants.WRIST_START_POS);
                hand.setPosition(RobotConstants.HAND_START_POS);
            }
        }

        if(gamepad2.a){
            controlState = Gamepad2ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLength(slides, 10),
                    new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*60/180)
                    );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_START_POS);
            hand.setPosition(RobotConstants.HAND_START_POS);
        }
        if(gamepad2.x){
            controlState = Gamepad2ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLength(slides, Math.sqrt(Math.pow(11.5, 2) + Math.pow(12, 2))),
                    new SequentialAction(
                            new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI * 110 / 180),
                            new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 180 / 180, Math.atan2(11.5, 12) + Math.PI * 90 / 180))),
                            new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(11.5, 12) + Math.PI * 90 / 180)
                    )
            );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
        }
        if(gamepad2.y){
            controlState = Gamepad2ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLength(slides, 40),
                    new SequentialAction(
                            new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI*110/180),
                            new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 180 / 180, Math.PI * 165 / 180))),
                            new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*165/180)
                    )
            );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_START_POS);
        }

        if(gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0){
            controlState = Gamepad2ControlState.MANUAL_CONTROL;
        }

        //Drive Controls
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

        //Servo Controls
        if(gamepad2.dpad_right){
            wrist.setPosition(RobotConstants.WRIST_START_POS);
        }
        else if(gamepad2.dpad_left){
            wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
        }

        if(g2b.activated(gamepad2.b) || (g1b.activated(gamepad1.b) && controlState.equals(Gamepad2ControlState.GRAB))) {
            if(Math.abs(fingers.getPosition() - RobotConstants.FINGER_CLOSE_POS) < 0.02){
                fingers.setPosition(RobotConstants.FINGER_OPEN_POS);
            }
            else{
                fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            }
        }

        //Controls for basically everything else
        if(controlState.equals(Gamepad2ControlState.GRAB)) {
            targetGroundDistance += gamepad1.dpad_up ? (gamepad1.right_trigger > 0.8 ? 0.3 : 0.6) : 0;
            targetGroundDistance -= gamepad1.dpad_down ? (gamepad1.right_trigger > 0.8 ? 0.3 : 0.6) : 0;
            targetGroundDistance = Math.max(targetGroundDistance, 6.0);
            targetGroundDistance = Math.min(targetGroundDistance, RobotConstants.MAX_GROUND_DISTANCE);

            clawGrabHeight += gamepad1.y ? (gamepad1.right_trigger > 0.8 ? 0.5 : 1) : 0;
            clawGrabHeight -= gamepad1.a ? (gamepad1.right_trigger > 0.8 ? 0.5 : 1) : 0;
            clawGrabHeight = Math.max(clawGrabHeight, RobotConstants.MIN_GROUND_HEIGHT);
            clawGrabHeight = Math.min(clawGrabHeight, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT-0.01);

            targetTurretAngle = Math.atan(targetGroundDistance / (RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight));
            targetTurretAngle = Math.min(turrets.getAngleRadians() + Math.PI / 12, targetTurretAngle);
            targetTurretAngle = Math.max(turrets.getAngleRadians() - Math.PI / 12, targetTurretAngle);
            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

            turrets.setAngleRadians(targetTurretAngle);
            slides.setLength(targetSlideLength);
            hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));

            telemetry.addData("Target Turret Angle: ", targetTurretAngle*180/Math.PI);
            telemetry.addData("Target Hand Pos: ", handPosFromAngle(Math.PI*270/180, targetTurretAngle));
        }
        if(controlState.equals(Gamepad2ControlState.PRESET)){
            double futureGroundDistance = Math.sin(turrets.getAngleRadians()) * slides.getLength();
            if(futureGroundDistance > RobotConstants.MAX_PRESET_GROUND_DISTANCE){
                slides.setLength(RobotConstants.MAX_PRESET_GROUND_DISTANCE / Math.sin(turrets.getAngleRadians()));
            }

            TelemetryPacket packet = new TelemetryPacket();
            if(!presetAction.run(packet)) {
                presetAction = new NullAction();
            }
        }
        if(controlState.equals(Gamepad2ControlState.MANUAL_CONTROL)){
            targetSlideLength = 10000 * -gamepad2.right_stick_y;
            targetSlideVelocity = slideCtv.targetSpeedFromJoysticks(-gamepad2.right_stick_y) * 384.5*435.0/60.0;

            targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
            targetTurretAngle = Math.min(turrets.getAngleRadians() + Math.PI / 24, targetTurretAngle);
            targetTurretAngle = Math.max(turrets.getAngleRadians() - Math.PI / 24, targetTurretAngle);

            double futureGroundDistance = Math.sin(targetTurretAngle) * slides.getLength();
            if(futureGroundDistance > RobotConstants.MAX_GROUND_DISTANCE){
                targetSlideLength = RobotConstants.MAX_GROUND_DISTANCE / Math.sin(targetTurretAngle);
                targetSlideVelocity = 384.5*435.0/60.0;
            }

            double futureGroundHeight = RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(targetTurretAngle) * slides.getLength();
            if(futureGroundHeight < RobotConstants.MIN_GROUND_HEIGHT){
                targetTurretAngle = Math.acos((RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - RobotConstants.MIN_GROUND_HEIGHT) / slides.getLength());
                targetTurretAngle = Math.min(turrets.getAngleRadians() + Math.PI / 12, targetTurretAngle);
                targetTurretAngle = Math.max(turrets.getAngleRadians() - Math.PI / 12, targetTurretAngle);
            }

            turrets.setAngleRadians(targetTurretAngle);
            slides.setLength(targetSlideLength, targetSlideVelocity);
        }

        telemetry.update();
    }
}