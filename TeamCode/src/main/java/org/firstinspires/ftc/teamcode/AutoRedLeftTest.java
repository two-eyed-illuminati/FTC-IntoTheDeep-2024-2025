package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AutoTunables;
import org.firstinspires.ftc.teamcode.utils.DualSlide;
import org.firstinspires.ftc.teamcode.utils.DualSlideSetLength;
import org.firstinspires.ftc.teamcode.utils.DualTurret;
import org.firstinspires.ftc.teamcode.utils.DualTurretAction;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

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
                new DualSlideSetLength(slides, Math.sqrt(Math.pow(AutoTunables.SPECIMEN_HEIGHT, 2) + Math.pow(12, 2))),
                new SequentialAction(
                        new DualTurretAction(turrets).setMode(DualTurretAction.Mode.GO_ABOVE).setTargetAngleRadians(Math.PI * 110 / 180),
                        new InstantAction(() -> hand.setPosition(handPosFromAngle(Math.PI * 180 / 180, Math.atan2(AutoTunables.SPECIMEN_HEIGHT, 12) + Math.PI * 90 / 180))),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(AutoTunables.SPECIMEN_HEIGHT, 12) + Math.PI * 90 / 180)
                )
        );
        return presetAction;
    }
    public Action getSamplePreset(){
        double targetGroundDistance = AutoTunables.SAMPLE_FORWARD - AutoTunables.SPECIMEN_FORWARD;

        double clawGrabHeight = RobotConstants.MIN_GRAB_HEIGHT;

        double targetTurretAngle = Math.atan2(targetGroundDistance, RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight);
        double targetSlideLength = Math.sqrt(Math.pow(targetGroundDistance, 2) + Math.pow(RobotConstants.SLIDE_PIVOT_GROUND_HEIGHT - clawGrabHeight, 2));

        Action presetAction = new ParallelAction(
          new DualTurretAction(turrets).setTargetAngleRadians(targetTurretAngle),
          new DualSlideSetLength(slides, targetSlideLength),
          new InstantAction(() -> {
              fingers.setPosition(AutoTunables.GRAB_FINGER_OPEN_POS);
              wrist.setPosition(RobotConstants.WRIST_PERPEN_POS);
              hand.setPosition(handPosFromAngle(Math.PI*270/180, targetTurretAngle));
          })
        );

        return presetAction;
    }
    public Action resetPreset(){
        Action presetAction = new ParallelAction(
                new DualSlideSetLength(slides, 10),
                new DualTurretAction(turrets).setTargetAngleRadians(Math.PI*120/180)
        );
        fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
        wrist.setPosition(RobotConstants.WRIST_START_POS);
        hand.setPosition(RobotConstants.HAND_START_POS);
        return presetAction;
    }
    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);

        DcMotorEx turretLeftMotor = hardwareMap.get(DcMotorEx.class, "turretLeft"); turretLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx turretRightMotor = hardwareMap.get(DcMotorEx.class, "turretRight");
        turrets = new DualTurret(turretLeftMotor, turretRightMotor);

        DcMotorEx slideLeftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft"); slideLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx slideRightMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        slides = new DualSlide(slideLeftMotor, slideRightMotor);

        fingers = hardwareMap.get(Servo.class, "fingers"); fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
        hand = hardwareMap.get(Servo.class, "hand"); hand.setPosition(RobotConstants.HAND_START_POS);
        wrist = hardwareMap.get(Servo.class, "wrist"); wrist.setPosition(RobotConstants.WRIST_START_POS);

        waitForStart();

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose).lineToX(AutoTunables.SPECIMEN_FORWARD);
        Action scoreSpecimen = new SequentialAction(
                new ParallelAction(
                        forward.build(),
                        specimenPreset()
                ),
                new SleepAction(AutoTunables.WAIT_TIME),
                new ParallelAction(
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.atan2(13, 12) + Math.PI * 90 / 180),
                        new DualSlideSetLength(slides, Math.sqrt(Math.pow(13, 2) + Math.pow(12, 2)))
                ),
                new InstantAction(() -> {fingers.setPosition(RobotConstants.FINGER_OPEN_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME),
                new ParallelAction(
                        new InstantAction(() -> {hand.setPosition(RobotConstants.HAND_START_POS);}),
                        new DualSlideSetLength(slides, 10.5),
                        new DualTurretAction(turrets).setTargetAngleRadians(Math.PI * 60 / 180)
                ),
                new InstantAction(() -> {
                    fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);
                    wrist.setPosition(RobotConstants.WRIST_START_POS);
                })
        );
        TrajectoryActionBuilder moveToSamples = forward.endTrajectory().fresh().lineToY(24+9);
        Action getSample = new SequentialAction(
                getSamplePreset(),
                new SleepAction(AutoTunables.WAIT_TIME),
                new InstantAction(() -> {fingers.setPosition(RobotConstants.FINGER_CLOSE_POS);}),
                new SleepAction(AutoTunables.WAIT_TIME),
                resetPreset()
        );
        TrajectoryActionBuilder moveToBasket = moveToSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(135)), Math.toRadians(135));

        Actions.runBlocking(new SequentialAction(
                scoreSpecimen,
                moveToSamples.build(),
                getSample
        ));
        Actions.runBlocking(resetPreset());
    }
}
