package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AutoTunables;
import org.firstinspires.ftc.teamcode.utils.DualSlide;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLengthWithLimit;
import org.firstinspires.ftc.teamcode.utils.DualTurret;
import org.firstinspires.ftc.teamcode.utils.DualTurretAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Transfer;
import org.opencv.core.Mat;

@Autonomous
public class AutoLeftSample extends LinearOpMode {
    MecanumDrive drive;
    DualSlide slides;
    DualTurret turrets;
    Servo fingers, hand, wrist;
    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI*3/2-turretAngle)+Math.PI- RobotConstants.HAND_START_ANGLE)*(RobotConstants.HAND_PARALLEL_POS-RobotConstants.HAND_START_POS)/(Math.PI-RobotConstants.HAND_START_ANGLE)+RobotConstants.HAND_START_POS;
    }
    public double findTargetTurretAngle(double groundDistance, double height){
        return Math.atan2(groundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - height);
    }
    public double findTargetSlideLength(double groundDistance, double height){
        return Math.sqrt(Math.pow(groundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - height, 2));
    }
    public Action setArmPos(double groundDistance, double height, boolean limit){
        double targetTurretAngle = findTargetTurretAngle(groundDistance, height);
        double targetSlideLength = findTargetSlideLength(groundDistance, height);
        if(limit){
            return new ParallelAction(
                    new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, targetSlideLength), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                    new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle)
            );
        }
        else{
            return new ParallelAction(
                    new DualSlideSetLength(slides, targetSlideLength),
                    new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle)
            );
        }
    }
    public Action grab(){
        return new SequentialAction(
                new InstantAction(() -> fingers.setPosition(RobotConstants.FINGER_CLOSE_POS)),
                new SleepAction(AutoTunables.WAIT_TIME)
        );
    }
    public Action release(){
        return new SequentialAction(
                new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                new SleepAction(AutoTunables.WAIT_TIME)
        );
    }
    public Action highBasketPreset(){
        return new ParallelAction(
                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 36), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE+2),
                new SequentialAction(
                        new InstantAction(() -> {if(turrets.getAngleRadians() < Math.PI*110/180) hand.setPosition(handPosFromAngle(Math.PI * 145 / 180, Math.PI * 155 / 180));}),
                        new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI*110/180),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 145 / 180, Math.PI * 150 / 180))),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*150/180)
                ),
                new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS))
        );
    }
    public Action defaultPreset(double turretAngle, double slideLen){
        Action presetAction = new ParallelAction(
                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10.5), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI * 90 / 180)
        );
        double currGroundHeight = RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turretAngle) * slideLen;
        if(currGroundHeight > 15){
            //Wait for hand servo to come back if we're coming back from a basket
            presetAction = new SequentialAction(new SleepAction(0.5), presetAction);
        }
        presetAction = new ParallelAction(
                new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_START_POS)),
                new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*90/180, Math.PI*155/180))),
                presetAction
        );
        return presetAction;
    }
    public Action defaultPreset(double turretAngle, double slideLen, double handPos){
        Action presetAction = new ParallelAction(
                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 10.5), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI * 90 / 180)
        );
        double currGroundHeight = RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(turretAngle) * slideLen;
        if(currGroundHeight > 15){
            //Wait for hand servo to come back if we're coming back from a basket
            presetAction = new SequentialAction(new SleepAction(0.5), presetAction);
        }
        presetAction = new ParallelAction(
                new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_START_POS)),
                new InstantAction(() -> hand.setPosition(handPos)),
                presetAction
        );
        return presetAction;
    }
    @Override
    public void runOpMode() {
        //See meepmeep field map for coordinate system
        //Heading 0 faces away from audience, towards positive x
        //Heading -90 faces towards red alliance area, towards negative y
        Pose2d initialPose = new Pose2d(9+22, 62, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParameters);
        imu.resetYaw();

        DcMotorEx turretLeftMotor = hardwareMap.get(DcMotorEx.class, "turretLeft"); turretLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turrets = new DualTurret(turretLeftMotor, turretRightMotor, true);

        DcMotorEx slideLeftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft"); slideLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx slideRightMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        slides = new DualSlide(slideLeftMotor, slideRightMotor, true);

        fingers = hardwareMap.get(Servo.class, "fingers"); fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
        hand = hardwareMap.get(Servo.class, "hand"); hand.setPosition(RobotConstants.HAND_START_POS);
        wrist = hardwareMap.get(Servo.class, "wrist"); wrist.setPosition(RobotConstants.WRIST_START_POS);

        Transfer.ranAuto = true;
        Transfer.imu = imu;

        telemetry.addData("Slide Length", slides.getLength());
        telemetry.addData("Turret Angle", turrets.getAngleDegrees());
        telemetry.addData("IMU Angle", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        TrajectoryActionBuilder toPreloadBasket = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-90))
                .lineToY(AutoTunables.BASKET_Y)
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.BASKET_X)
                .turnTo(Math.toRadians(50));
        TrajectoryActionBuilder moveToSamples = toPreloadBasket.endTrajectory().fresh()
                .turnTo(Math.toRadians(-90))
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.SAMPLE_X)
                .setTangent(Math.toRadians(90))
                .lineToY(AutoTunables.SAMPLE_Y);
        TrajectoryActionBuilder moveToBasket = moveToSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(AutoTunables.BASKET_Y)
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.BASKET_X)
                .turnTo(Math.toRadians(45));
        TrajectoryActionBuilder goToSample2FromBasket = moveToBasket.endTrajectory().fresh()
                .turnTo(Math.toRadians(-90))
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.SAMPLE_X + 10)
                .setTangent(Math.toRadians(90))
                .lineToY(AutoTunables.SAMPLE_Y);
        TrajectoryActionBuilder moveToBasket2 = goToSample2FromBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(AutoTunables.BASKET_Y)
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.BASKET_X)
                .turnTo(Math.toRadians(45));
        TrajectoryActionBuilder goToSample3FromBasket = moveToBasket2.endTrajectory().fresh()
                .turnTo(Math.toRadians(-90))
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.SAMPLE_X + 13.5)
                .turnTo(Math.toRadians(-75));
        TrajectoryActionBuilder moveToBasket3Turn = goToSample3FromBasket.endTrajectory().fresh()
                .turnTo(Math.toRadians(-90));
        TrajectoryActionBuilder moveToBasket3 = moveToBasket3Turn.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(AutoTunables.BASKET_Y)
                .setTangent(Math.toRadians(0))
                .lineToX(AutoTunables.BASKET_X)
                .turnTo(Math.toRadians(45));
        TrajectoryActionBuilder toSubmersible = moveToBasket3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.END_X, AutoTunables.END_Y, Math.toRadians(-180)), Math.toRadians(-180));

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            highBasketPreset(),
                            toPreloadBasket.build()
                        ),
                        release(),
                        new ParallelAction(
                                moveToSamples.build(),
                                defaultPreset(Math.toRadians(150), 36)
                        ),
                        new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS)),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(23.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                        new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                        setArmPos(23, AutoTunables.SAMPLE_GRAB_HEIGHT+3, false),
                        setArmPos(23, AutoTunables.SAMPLE_GRAB_HEIGHT, false),
                        new SleepAction(AutoTunables.WAIT_TIME*2),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        new ParallelAction(
                                highBasketPreset(),
                                new SequentialAction(
                                        new SleepAction(AutoTunables.WAIT_TIME*2),
                                        moveToBasket.build()
                                )
                        ),
                        release(),
                        new ParallelAction(
                                goToSample2FromBasket.build(),
                                defaultPreset(Math.toRadians(150), 36)
                        ),
                        new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS)),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(23.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                        new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                        setArmPos(23, AutoTunables.SAMPLE_GRAB_HEIGHT+3, false),
                        setArmPos(23, AutoTunables.SAMPLE_GRAB_HEIGHT, false),
                        new SleepAction(AutoTunables.WAIT_TIME*2),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        new ParallelAction(
                                highBasketPreset(),
                                moveToBasket2.build()
                        ),
                        release(),
                        new ParallelAction(
                                goToSample3FromBasket.build(),
                                new SequentialAction(
                                    defaultPreset(Math.toRadians(150), 36),
                                    new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS-0.05)),
                                    new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(23.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                                    new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS))
                                )
                        ),
                        setArmPos(AutoTunables.SAMPLE_3_EXT_LEN, AutoTunables.SAMPLE_GRAB_HEIGHT+1.25, false),
                        setArmPos(AutoTunables.SAMPLE_3_EXT_LEN, AutoTunables.SAMPLE_GRAB_HEIGHT, false),
                        new SleepAction(AutoTunables.WAIT_TIME*2),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        new ParallelAction(
                                setArmPos(AutoTunables.SAMPLE_3_EXT_LEN, AutoTunables.SAMPLE_GRAB_HEIGHT+3, false),
                                moveToBasket3Turn.build()
                        ),
                        new ParallelAction(
                                highBasketPreset(),
                                moveToBasket3.build()
                        ),
                        release(),
                        new ParallelAction(
                            toSubmersible.build(),
                            defaultPreset(Math.toRadians(150), 36, RobotConstants.HAND_START_POS)
                        )
                )
        );
    }
}