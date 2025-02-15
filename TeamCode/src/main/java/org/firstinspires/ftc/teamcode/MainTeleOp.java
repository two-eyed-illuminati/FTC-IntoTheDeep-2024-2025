package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ControlsToValues;
import org.firstinspires.ftc.teamcode.utils.Drive;
import org.firstinspires.ftc.teamcode.utils.DualSlide;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLengthWithLimit;
import org.firstinspires.ftc.teamcode.utils.DualTurret;
import org.firstinspires.ftc.teamcode.utils.DualTurretAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.ToggleButton;
import org.firstinspires.ftc.teamcode.utils.Transfer;

import java.util.List;

enum ControlState {
    GRAB, MANUAL_CONTROL, PRESET
}

@TeleOp
public class MainTeleOp extends OpMode{
    Drive fod;
    DualTurret turrets; DualSlide slides;
    ControlsToValues slideCtv = new ControlsToValues();
    DcMotorEx fl, fr, bl, br;
    Servo fingers, hand, wrist;
    ControlState controlState = ControlState.PRESET;
    ToggleButton g1lT = new ToggleButton(); ToggleButton g2b = new ToggleButton(); ToggleButton g1b = new ToggleButton();
    ToggleButton g1dR = new ToggleButton(); ToggleButton g1dL = new ToggleButton();
    double targetGroundDistance; double clawGrabHeight;
    double targetSlideLength = 10.5; double targetSlideVelocity = 0;
    double targetTurretAngle = Math.PI*120/180;
    Action presetAction = new NullAction();
    List<LynxModule> allHubs;
    ElapsedTime elpasedTime;

    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI*3/2-turretAngle)+Math.PI-RobotConstants.HAND_START_ANGLE)*(RobotConstants.HAND_PARALLEL_POS-RobotConstants.HAND_START_POS)/(Math.PI-RobotConstants.HAND_START_ANGLE)+RobotConstants.HAND_START_POS;
    }
    @Override
    public void init() {
        telemetry.addLine("REMINDER: Lift turret up and retract slides");
        telemetry.update();

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        IMU imu;
        if(!Transfer.ranAuto) {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(imuParameters);
            fod = new Drive(fl, fr, bl, br, imu);
            fod.resetImu();
        }
        else {
            imu = Transfer.imu;
            Transfer.imu = null;
            telemetry.addLine("Found transferred imu");
            telemetry.addData("Angle:", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            fod = new Drive(fl, fr, bl, br, imu);
        }

        DcMotorEx turretLeftMotor = hardwareMap.get(DcMotorEx.class, "turretLeft");
        turretLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turrets = new DualTurret(turretLeftMotor, turretRightMotor, !Transfer.ranAuto);

        DcMotorEx slideLeftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft");
        slideLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx slideRightMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        slides = new DualSlide(slideLeftMotor, slideRightMotor, !Transfer.ranAuto);

        slideCtv.cubicLowerSpeedValue = 0.2;

        fingers = hardwareMap.get(Servo.class, "fingers");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");

        controlState = ControlState.PRESET;
        presetAction = new ParallelAction(
                new InstantAction(() -> {
                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                    hand.setPosition(RobotConstants.HAND_START_POS);
                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                }),
                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10.5), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*60/180)
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Transfer.ranAuto = false;
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        telemetry.addData("Ping (ms): ", elpasedTime.milliseconds());
        elpasedTime.reset();
        telemetry.addData("MODE: ", controlState.name());
        telemetry.addData("Turret Angle: ", turrets.getAngleDegrees());
        telemetry.addData("Slide Length: ", slides.getLength());
        telemetry.addData("Hand Pos: ", hand.getPosition());
        telemetry.addData("Wrist Pos: ", wrist.getPosition());
        telemetry.addData("Fingers Pos: ", fingers.getPosition());

        //Switching Control Modes
        if(g1lT.activated(gamepad1.left_trigger > 0.8)) {
            if(!controlState.equals(ControlState.GRAB)) {
                double currGroundHeight = RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turrets.getAngleRadians()) * slides.getLength();
                if(currGroundHeight < 15){ //Make sure driver doesn't press this right after scoring in basket, otherwise could break servo
                    controlState = ControlState.GRAB;
                    targetGroundDistance = 14.0;
                    clawGrabHeight = RobotConstants.MAX_GRAB_HEIGHT;
                    fingers.setPosition(RobotConstants.FINGER_OPEN_POS);
                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                }
                else{
                    controlState = ControlState.PRESET;
                    targetGroundDistance = 14.0;
                    clawGrabHeight = RobotConstants.MAX_GRAB_HEIGHT;
                    targetTurretAngle = Math.atan2(targetGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight);
                    targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

                    presetAction = new SequentialAction(
                            new SleepAction(0.5), //wait for servos to move
                            new ParallelAction(
                                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, targetSlideLength), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                                new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle)
                            ),
                            new InstantAction(() -> {
                                controlState = ControlState.GRAB;
                                fingers.setPosition(RobotConstants.FINGER_OPEN_POS);
                                wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
                            })
                    );
                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                    hand.setPosition(RobotConstants.HAND_START_POS);
                }
            }
            else {
                controlState = ControlState.PRESET;
                double currGroundDistance = Math.sin(turrets.getAngleRadians()) * slides.getLength();
                targetTurretAngle = Math.atan2(currGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - (RobotConstants.MAX_GRAB_HEIGHT - 2));
                presetAction = new SequentialAction(
                        new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle).setMargin(Math.PI * 4 / 180),
                        new ParallelAction(
                            new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*110/180, targetTurretAngle))),
                            new SleepAction(0.35) //wait for servos to move
                        ),
                        new ParallelAction(
                            new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10.5), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                            new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*60/180)
                        ),
                        new InstantAction(() -> hand.setPosition(RobotConstants.HAND_START_POS))
                );
                fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                wrist.setPosition(RobotConstants.WRIST_START_POS);
            }
        }

        if(gamepad2.a && !controlState.equals(ControlState.GRAB)){
            controlState = ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10.5), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                    new DualTurretAction(turrets).setTargetAngleRadians(Math.PI * 60 / 180)
            );
            double currGroundHeight = RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turrets.getAngleRadians()) * slides.getLength();
            if(currGroundHeight > 15){
                //Wait for hand servo to come back if we're coming back from a basket
                presetAction = new SequentialAction(new SleepAction(0.5), presetAction);
            }
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_START_POS);
            hand.setPosition(RobotConstants.HAND_START_POS);
        }
        if(gamepad2.x && !controlState.equals(ControlState.GRAB)){
            controlState = ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, Math.sqrt(Math.pow(11.5, 2) + Math.pow(12, 2))), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                    new SequentialAction(
                            new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI * 110 / 180),
                            new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 180 / 180, Math.atan2(11.5, 12) + Math.PI * 90 / 180))),
                            new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(11.5, 12) + Math.PI * 90 / 180)
                    )
            );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
        }
        if(gamepad2.y && !controlState.equals(ControlState.GRAB)){
            controlState = ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 36), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                    new SequentialAction(
                            new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI*110/180),
                            new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 145 / 180, Math.PI * 155 / 180))),
                            new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*155/180)
                    )
            );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
        }
        if(gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8 && gamepad2.left_bumper && gamepad2.right_bumper){
            controlState = ControlState.PRESET;
            presetAction = new ParallelAction(
                    new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                    new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*120/180)
            );
            fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            wrist.setPosition(RobotConstants.WRIST_START_POS);
            hand.setPosition(RobotConstants.HAND_START_POS);
        }

        //Should rarely, if ever, be used in an actual match
        if(gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0){
            controlState = ControlState.MANUAL_CONTROL;
        }

        //Drive Controls
        double[] motorPowers = new double[10];
        double xDrive = gamepad1.left_stick_x * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
        double yDrive = -gamepad1.left_stick_y * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
        double rotation = gamepad1.right_stick_x * (gamepad1.right_trigger > 0.8 ? 0.5 : 1);
        motorPowers = fod.driveFieldCentric(xDrive, yDrive, rotation);
        telemetry.addData("xDrive", xDrive);
        telemetry.addData("yDrive", yDrive);
        telemetry.addData("rotation", rotation);
        telemetry.addData("fl", motorPowers[0]);
        telemetry.addData("fr", motorPowers[1]);
        telemetry.addData("bl", motorPowers[2]);
        telemetry.addData("br", motorPowers[3]);

        //Servo Controls
        if(gamepad2.dpad_right || (gamepad1.dpad_right && controlState.equals(ControlState.GRAB))){
            double newPos = Math.min(RobotConstants.WRIST_START_POS+RobotConstants.WRIST_START_POS-RobotConstants.WRIST_PERPEN_POS, wrist.getPosition()+0.02);
            wrist.setPosition(newPos);
        }
        else if(gamepad2.dpad_left || (gamepad1.dpad_left && controlState.equals(ControlState.GRAB))){
            double newPos = Math.max(RobotConstants.WRIST_PERPEN_POS, wrist.getPosition()-0.02);
            wrist.setPosition(newPos);
        }

        if(g2b.activated(gamepad2.b) || (g1b.activated(gamepad1.b) && controlState.equals(ControlState.GRAB))) {
            if(Math.abs(fingers.getPosition() - RobotConstants.FINGER_CLOSE_POS) < 0.02){
                fingers.setPosition(RobotConstants.FINGER_OPEN_POS);
            }
            else{
                fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
            }
        }

        //Controls for basically everything else
        if(controlState.equals(ControlState.GRAB)) {
            targetGroundDistance += gamepad1.dpad_up ? (gamepad1.right_trigger > 0.8 ? 0.3 : 0.6) : 0;
            targetGroundDistance -= gamepad1.dpad_down ? (gamepad1.right_trigger > 0.8 ? 0.3 : 0.6) : 0;
            targetGroundDistance = Math.max(targetGroundDistance, 6.0);
            targetGroundDistance = Math.min(targetGroundDistance, RobotConstants.MAX_GROUND_DISTANCE);

            clawGrabHeight += gamepad1.y ? (gamepad1.right_trigger > 0.8 ? 0.2 : 0.4) : 0;
            clawGrabHeight -= gamepad1.a ? (gamepad1.right_trigger > 0.8 ? 0.2 : 0.4) : 0;
            clawGrabHeight = Math.max(clawGrabHeight, RobotConstants.MIN_GRAB_HEIGHT);
            clawGrabHeight = Math.min(clawGrabHeight, RobotConstants.MAX_GRAB_HEIGHT);

            targetTurretAngle = Math.atan2(targetGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight);
            targetTurretAngle = Math.min(turrets.getAngleRadians() + Math.PI / 12, targetTurretAngle);
            targetTurretAngle = Math.max(turrets.getAngleRadians() - Math.PI / 12, targetTurretAngle);
            targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

            turrets.setAngleRadians(targetTurretAngle);
            slides.setLength(targetSlideLength);
            hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));

            telemetry.addData("Target Ground Distance: ", targetGroundDistance);
            telemetry.addData("Target Ground Height: ", clawGrabHeight);
        }
        if(controlState.equals(ControlState.PRESET)){
            TelemetryPacket packet = new TelemetryPacket();
            if(!presetAction.run(packet)) {
                telemetry.addLine("DONE");
                presetAction = new NullAction();
            }
        }
        //MANUAL_CONTROL should rarely, if ever, be used in an actual match
        if(controlState.equals(ControlState.MANUAL_CONTROL)){
            targetSlideLength = Math.abs(gamepad2.right_stick_y) > 0.05 ? 10000 * -Math.signum(gamepad2.right_stick_y) :
                                Math.abs(targetSlideLength-slides.getLength()) > 1 ? slides.getLength() : targetSlideLength;
            targetSlideVelocity = (Math.abs(gamepad2.right_stick_y) > 0.05 ?
                    slideCtv.targetSpeedFromJoysticks(-gamepad2.right_stick_y) : 1) *
                    384.5*435.0/60.0;

            targetTurretAngle -= gamepad2.left_stick_y * gamepad2.left_stick_y * 0.25 * Math.signum(gamepad2.left_stick_y);
            targetTurretAngle = Math.min(turrets.getAngleRadians() + Math.PI / 24, targetTurretAngle);
            targetTurretAngle = Math.max(turrets.getAngleRadians() - Math.PI / 24, targetTurretAngle);

            double futureGroundDistance = Math.sin(targetTurretAngle) * targetSlideLength;
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
