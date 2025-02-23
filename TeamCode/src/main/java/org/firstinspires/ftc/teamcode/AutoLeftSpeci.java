package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.utils.AutoTunables;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DualSlide;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLengthWithLimit;
import org.firstinspires.ftc.teamcode.utils.DualTurret;
import org.firstinspires.ftc.teamcode.utils.DualTurretAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Transfer;

@Autonomous
public class AutoLeftSpeci extends LinearOpMode {
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
                new InstantAction(() -> hand.setPosition(RobotConstants.HAND_START_POS)),
                presetAction
        );
        return presetAction;
    }
    @Override
    public void runOpMode() {
        //See meepmeep field map for coordinate system
        //Heading 0 faces away from audience, towards positive x
        //Heading -90 faces towards red alliance area, towards negative y
        Pose2d initialPose = new Pose2d(9, 63, Math.toRadians(-90));
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

        TrajectoryActionBuilder toSpecimen = drive.actionBuilder(initialPose).lineToY(AutoTunables.SPECIMEN_Y);
        TrajectoryActionBuilder moveToSamples = toSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_Y), 0);
        TrajectoryActionBuilder moveToBasket = moveToSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(45));
        TrajectoryActionBuilder goToSample2FromBasket = moveToBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(AutoTunables.SAMPLE_X + 10, AutoTunables.SAMPLE_Y, Math.toRadians(-90)), Math.toRadians(-90));
        TrajectoryActionBuilder moveToBasket2 = goToSample2FromBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(90));
        TrajectoryActionBuilder goToSample3FromBasket = moveToBasket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_3_Y, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .lineToX(AutoTunables.SAMPLE_3_X);
        TrajectoryActionBuilder moveToBasket3 = goToSample3FromBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(-180))
                .lineToX(AutoTunables.SAMPLE_X-3)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(45));
        TrajectoryActionBuilder toSubmersible = moveToBasket3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.SAMPLE_X-8, AutoTunables.END_Y, Math.toRadians(180)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(AutoTunables.END_X, AutoTunables.END_Y, Math.toRadians(-180)), Math.toRadians(-180));

        Actions.runBlocking(
                new SequentialAction(
                        toSpecimen.build(),
                        moveToSamples.build(),
                        new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS)),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(11.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                        new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        new ParallelAction(
                                setArmPos(11, AutoTunables.SAMPLE_GRAB_HEIGHT, false)
                        ),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        highBasketPreset(),
                        new ParallelAction(
                                moveToBasket.build()
                        ),
                        release(),
                        new ParallelAction(
                                goToSample2FromBasket.build(),
                                defaultPreset(Math.toRadians(150), 36)
                        ),
                        new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_PERPEN_POS)),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(11.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                        new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                        new SleepAction(AutoTunables.WAIT_TIME*7),
                        new ParallelAction(
                                setArmPos(11, AutoTunables.SAMPLE_GRAB_HEIGHT, false)
                        ),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        highBasketPreset(),
                        new ParallelAction(
                                moveToBasket2.build()
                        ),
                        release(),
                        new ParallelAction(
                                goToSample3FromBasket.build(),
                                new SequentialAction(
                                        defaultPreset(Math.toRadians(150), 36),
                                        new ParallelAction(
                                                setArmPos(11.0, AutoTunables.SAMPLE_GRAB_HEIGHT+2, false),
                                                new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(11.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                                                new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS))
                                        )
                                )
                        ),
                        new InstantAction(() -> wrist.setPosition(RobotConstants.WRIST_START_POS)),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI*290/180, findTargetTurretAngle(11.0, AutoTunables.SAMPLE_GRAB_HEIGHT)))),
                        new InstantAction(() -> fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS)),
                        new SleepAction(AutoTunables.WAIT_TIME*7),
                        new ParallelAction(
                                setArmPos(11, AutoTunables.SAMPLE_GRAB_HEIGHT, false)
                        ),
                        grab(),
                        new SleepAction(AutoTunables.WAIT_TIME),
                        new ParallelAction(
                                moveToBasket3.build(),
                                new SequentialAction(
                                    new SleepAction(0.6),
                                    highBasketPreset()
                                )
                        ),
                        release(),
                        new ParallelAction(
                            toSubmersible.build(),
                            new SequentialAction(
                                    defaultPreset(Math.toRadians(150), 36),
                                    setArmPos(18.0, 20.0, false)
                            )
                        )
                )
        );
    }
}