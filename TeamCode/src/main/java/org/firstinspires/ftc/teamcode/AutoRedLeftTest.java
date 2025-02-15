package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Autonomous
public class AutoRedLeftTest extends LinearOpMode {
    MecanumDrive drive;
    DualSlide slides;
    DualTurret turrets;
    Servo fingers, hand, wrist;
    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI*3/2-turretAngle)+Math.PI-RobotConstants.HAND_START_ANGLE)*(RobotConstants.HAND_PARALLEL_POS-RobotConstants.HAND_START_POS)/(Math.PI-RobotConstants.HAND_START_ANGLE)+RobotConstants.HAND_START_POS;
    }
    public Action specimenPreset(){
        Action presetAction = new ParallelAction(
                new InstantAction(() -> {
                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                    wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
                }),
                new DualSlideSetLength(slides, Math.sqrt(Math.pow(AutoTunables.SPECIMEN_START_HEIGHT, 2) + Math.pow(12, 2))),
                new SequentialAction(
                        new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI * 110 / 180),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 180 / 180, Math.atan2(AutoTunables.SPECIMEN_START_HEIGHT, 12) + Math.PI * 90 / 180))),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(AutoTunables.SPECIMEN_START_HEIGHT, 12) + Math.PI * 90 / 180)
                )
        );
        return presetAction;
    }
    public Action speciDown(){
        class speciDownAction implements Action{
            DualTurretAction turretAction;
            DualSlideSetLengthWithLimit slidesAction;
            speciDownAction(){
                this.turretAction = new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(AutoTunables.SPECIMEN_END_HEIGHT, 12) + Math.PI * 90 / 180);
                this.slidesAction = new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 36, 384.5*435.0/60.0), turrets, 12);
            }
            public boolean run(TelemetryPacket packet){
                slidesAction.run(packet);
                return turretAction.run(packet);
            }
        }

        return new speciDownAction();
    }
    public Action basketPreset(){
        Action presetAction = new ParallelAction(
                new InstantAction(() -> {
                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                    wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
                }),
                new DualSlideSetLengthWithLimit(new DualSlideSetLength(slides, 36), turrets, RobotConstants.MAX_PRESET_GROUND_DISTANCE),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*155/180)
        );
        return presetAction;
    }
    public Action getSamplePreset(boolean isSample3){
        double targetGroundDistance = 15;

        double clawGrabHeight = AutoTunables.SAMPLE_GRAB_HEIGHT;

        double targetTurretAngle = Math.atan2(targetGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight);
        double targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

        Action presetAction = new ParallelAction(
          new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle),
          new DualSlideSetLength(slides, targetSlideLength),
          new InstantAction(() -> {
              fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS);
              wrist.setPosition(isSample3 ? RobotConstants.WRIST_START_POS : RobotConstants.WRIST_PERPEN_POS);
              hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));
          })
        );

        return presetAction;
    }
    public Action prepGetSamplePreset(boolean isSample3){
        double targetGroundDistance = 15;

        double clawGrabHeight = RobotConstants.MAX_GRAB_HEIGHT;

        double targetTurretAngle = Math.atan2(targetGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight);
        double targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

        Action presetAction = new ParallelAction(
                new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle),
                new DualSlideSetLength(slides, targetSlideLength),
                new InstantAction(() -> {
                    wrist.setPosition(isSample3 ? RobotConstants.WRIST_START_POS : RobotConstants.WRIST_PERPEN_POS);
                    hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));
                })
        );

        return presetAction;
    }
    public Action resetPreset(){
        Action presetAction = new ParallelAction(
                new DualSlideSetLength(slides, 11.5),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*120/180),
                new InstantAction(() -> {
                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                    hand.setPosition(RobotConstants.HAND_START_POS);
                })
        );
        return presetAction;
    }
    public Action getSampleAction(boolean isSample3){
        return new SequentialAction(
                getSamplePreset(isSample3),
                new SleepAction(AutoTunables.WAIT_TIME),
                new InstantAction(() -> {fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME*2)
        );
    }
    public Action highBucketScoreAction(){
        return new SequentialAction(
                new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 145 / 180, Math.PI * 155 / 180))),
                new SleepAction(AutoTunables.WAIT_TIME),
                new InstantAction(() -> {fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME),
                new InstantAction(() -> {hand.setPosition(RobotConstants.HAND_START_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME)
        );
    }
    @Override
    public void runOpMode(){
        MultipleTelemetry multTele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(0, 0, 0);
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

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose).lineToX(AutoTunables.SPECIMEN_FORWARD, new TranslationalVelConstraint(AutoTunables.SPECIMEN_FORWARD_SPEED*50));
        Action scoreSpecimen = new SequentialAction(
                new ParallelAction(
                        specimenPreset(),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME),
                                new InstantAction(() -> {
                                    drive.maxCorrectionTime = 3;
                                    drive.correctionMargin = 0.25;
                                }),
                                forward.build(),
                                new InstantAction(() -> {
                                    drive.maxCorrectionTime = 0.5;
                                    drive.correctionMargin = 0.5;
                                })
                        )
                ),
                new SleepAction(AutoTunables.WAIT_TIME),
                speciDown(),
                new InstantAction(() -> {fingers.setPosition(RobotConstants.FINGER_OPEN_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME)
        );
        TrajectoryActionBuilder moveToSamples = forward.endTrajectory().fresh().
                setTangent(Math.toRadians(90)).splineToConstantHeading(new Vector2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_Y), 0);
        TrajectoryActionBuilder moveToBasket = moveToSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(135)), Math.toRadians(135));
        TrajectoryActionBuilder goToSample2FromBasket = moveToBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_Y +11, Math.toRadians(0)), Math.toRadians(0));
        TrajectoryActionBuilder moveToBasket2 = goToSample2FromBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(135)), Math.toRadians(135));
        TrajectoryActionBuilder goToSample3FromBasketPart1 = moveToBasket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(AutoTunables.SAMPLE_3_X, AutoTunables.SAMPLE_Y, Math.toRadians(90)), Math.toRadians(0));
        TrajectoryActionBuilder goToSample3FromBasketPart2 = goToSample3FromBasketPart1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(AutoTunables.SAMPLE_3_Y);
        TrajectoryActionBuilder moveToBasket3 = goToSample3FromBasketPart2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(AutoTunables.SAMPLE_Y)
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(135)), Math.toRadians(135));
        TrajectoryActionBuilder end = moveToBasket3.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(AutoTunables.END_X, AutoTunables.SAMPLE_3_Y, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.END_X, AutoTunables.END_Y, Math.toRadians(-90)), Math.toRadians(-90));

        Actions.runBlocking(new SequentialAction(
                //Score preload specimen
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 0);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                scoreSpecimen,
                //Move to samples
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 1);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        new DualSlideSetLength(slides, 11.5),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.PI * 60 / 180),
                        new InstantAction(() -> {hand.setPosition(RobotConstants.HAND_START_POS);}),
                        new SequentialAction(
                            new SleepAction(AutoTunables.WAIT_TIME),
                            new ParallelAction(
                                moveToSamples.build(),
                                new InstantAction(() -> {
                                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                                })
                            )
                        )
                ),
                //Sample #1
                getSampleAction(false),
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 2);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        basketPreset(),
                        new InstantAction(() -> {hand.setPosition(handPosFromAngle(Math.PI*90/180, Math.PI*155/180));}),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME*2),
                                moveToBasket.build()
                        )
                ),
                highBucketScoreAction(),
                //Sample #2
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 3);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        goToSample2FromBasket.build(),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME),
                                new InstantAction(() -> {fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS);}),
                                prepGetSamplePreset(false)
                        )
                ),
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 4);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                getSampleAction(false),
                new ParallelAction(
                        basketPreset(),
                        new InstantAction(() -> {hand.setPosition(handPosFromAngle(Math.PI*90/180, Math.PI*155/180));}),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME*2),
                                moveToBasket2.build()
                        )
                ),
                highBucketScoreAction(),
                //Sample #3
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 5);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        new SequentialAction(
                                goToSample3FromBasketPart1.build(),
                                new SleepAction(AutoTunables.WAIT_TIME),
                                goToSample3FromBasketPart2.build()
                        ),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME),
                                resetPreset(),
                                new ParallelAction(
                                    prepGetSamplePreset(true),
                                    new InstantAction(() -> {fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS);})
                                )
                        )
                ),
                getSampleAction(true),
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 6);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        new SequentialAction(
                                prepGetSamplePreset(true),
                                new SleepAction(AutoTunables.WAIT_TIME),
                                new ParallelAction(
                                        new InstantAction(() -> {hand.setPosition(handPosFromAngle(Math.PI*90/180, Math.PI*155/180));}),
                                        basketPreset()
                                )
                        ),
                        moveToBasket3.build()
                ),
                highBucketScoreAction(),
                new InstantAction(() -> {
                    TelemetryPacket p = new TelemetryPacket();
                    p.put("State", 7);
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }),
                new ParallelAction(
                        end.build(),
                        new SequentialAction(
                                new SleepAction(AutoTunables.WAIT_TIME),
                                resetPreset(),
                                new ParallelAction(
                                        new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(5, 12) + Math.PI * 90 / 180),
                                        new DualSlideSetLength(slides, Math.sqrt(Math.pow(5, 2) + Math.pow(12, 2)))
                                )
                        )
                )
        ));
        TelemetryPacket p = new TelemetryPacket();
        p.put("State", 8);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
