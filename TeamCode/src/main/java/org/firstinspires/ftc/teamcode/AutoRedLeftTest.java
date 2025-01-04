package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.lang.reflect.Array;
import java.util.Arrays;

@Autonomous
public class AutoRedLeftTest extends LinearOpMode {
    class Preset implements Action{
        double turretAngle;
        double slideLength;
        double oldTurretAngle;
        double oldSlideLength;
        double handAngle;
        double wristPos;
        double fingersPos;
        public Preset(double turretAngle, double slideLength, double handAngle, double wristPos, double fingersPos){
            this.turretAngle = turretAngle;
            this.slideLength = slideLength;
            this.handAngle = handAngle;
            this.wristPos = wristPos;
            this.fingersPos = fingersPos;
            double oldTurretAngle = -1;
            double oldSlideLength = -1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            double currTurretAngle = turret.getAngleRadians();
            double currSlideLength = slide.getLength();
            double currTargetSlideLength = slideLength;
            double currTargetTurretAngle = turretAngle;
            double currTargetSlideVelocity = 537.7*312.0/60.0;
            double futureGroundDistance = Math.sin(currTurretAngle) * currSlideLength;
            if(futureGroundDistance > MAX_PRESET_GROUND_DISTANCE){
                currTargetSlideLength = MAX_PRESET_GROUND_DISTANCE / Math.sin(currTurretAngle);
                currTargetSlideVelocity = (537.7*312.0/60.0);
            }
            double futureGroundHeight = SLIDE_PIVOT_GROUND_HEIGHT - Math.cos(currTurretAngle) * currSlideLength;
            if(futureGroundHeight < MIN_GROUND_HEIGHT){
                currTargetTurretAngle = Math.acos((SLIDE_PIVOT_GROUND_HEIGHT - MIN_GROUND_HEIGHT) / currSlideLength);
                currTargetTurretAngle = Math.min(currTurretAngle + Math.PI / 12, currTargetTurretAngle);
                currTargetTurretAngle = Math.max(currTurretAngle - Math.PI / 12, currTargetTurretAngle);
            }
            turret.setAngleRadians(currTargetTurretAngle);
            slide.setLength(currTargetSlideLength, currTargetSlideVelocity);

            hand.setPosition(handPosFromAngle(handAngle, currTurretAngle));
            wrist.setPosition(wristPos);
            fingers.setPosition(fingersPos);

            if(oldTurretAngle == -1){
                oldTurretAngle = currTurretAngle;
                oldSlideLength = currSlideLength;
            }
//            telemetry.addData()
            else if(Math.abs(oldTurretAngle - currTurretAngle) < Math.PI*2/180 && Math.abs(oldSlideLength - currSlideLength) < 0.5){
                if(Math.abs(currTurretAngle - turretAngle) < Math.PI*2/180 && Math.abs(currSlideLength - slideLength) < 0.5){
                    return false;
                }
                oldTurretAngle = currTurretAngle;
                oldSlideLength = currSlideLength;
            }
            return true;
        }
    }
    class Grab implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            fingers.setPosition(FINGER_CLOSE_POS);
            return false;
        }
    }
    class Release implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            fingers.setPosition(FINGER_OPEN_POS);
            return false;
        }
    }
    class Forward implements Action{
        ElapsedTime timer = new ElapsedTime();
        boolean justStarted = true;
        public Forward(){
            timer.reset();
            justStarted = true;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if(justStarted){
                justStarted = false;
                timer.reset();
            }
            if(timer.milliseconds() < 1000){
                fl.setPower(0.75);
                fr.setPower(0.75);
                bl.setPower(0.75);
                br.setPower(0.75);
                return true;
            }
            else{
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                return false;
            }
        }
    }
    MecanumDrive drive;
    Slide slide;
    Turret turret;
    Servo hand, wrist, fingers;
    final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    final double MAX_GROUND_DISTANCE = 10+13; //Max ground length of slides
    final double MAX_PRESET_GROUND_DISTANCE = 10+13/2.0; //have a lower limit for max slide length for presets
    final double MIN_GROUND_HEIGHT = 7;
    final double FINGER_CLOSE_POS = 0.309;
    final double FINGER_OPEN_POS = 0.406;
    final double HAND_START_POS = 1.0;
    final double HAND_PARALLEL_POS = 0.4; //Pos where hand is parallel with slides
    final double HAND_START_ANGLE = Math.PI*0; //Positive acute angle between hand start and slides
    final double WRIST_START_POS = 0.1341;
    final double WRIST_PERPEN_POS = 0.32+WRIST_START_POS; //Pos where wrist is perpendicular to slides
    DcMotorEx fl, fr, bl, br;
    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI/2-turretAngle)+Math.PI*0/180)*(HAND_PARALLEL_POS-HAND_START_POS)/(Math.PI*180/180)+HAND_START_POS;
    }

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0, 0, 0);
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        drive = new MecanumDrive(hardwareMap, initialPose);

        slide = new Slide(hardwareMap.get(DcMotorEx.class, "liftLeft"));
        turret = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        fingers = hardwareMap.get(Servo.class, "fingers");

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose)
                .setTangent(0)
                .lineToX(26);
        Action forwardAction = forward.build();
        TrajectoryActionBuilder strafe = forward.endTrajectory().fresh().strafeTo(new Vector2d(26, 26), baseVelConstraint);
        Action strafeAction = strafe.build();
        TrajectoryActionBuilder turn = strafe.endTrajectory().fresh().splineToLinearHeading(new Pose2d(10, 72-10, 0), -3*Math.PI/2, baseVelConstraint);
        Action highChamberPreset = new Preset(
                Math.atan2(13, 8)+Math.PI*90/180,
                Math.sqrt(Math.pow(11.5, 2) + Math.pow(12, 2)),
                Math.PI*180/180, WRIST_PERPEN_POS, FINGER_CLOSE_POS);
        Action highBasketPreset = new Preset(
                Math.PI*180/180,
                36,
                Math.PI*0/180, WRIST_START_POS, FINGER_CLOSE_POS);
        Action defaultPreset = new Preset(
                Math.PI*50/180,
                11,
                HAND_START_POS, WRIST_START_POS, FINGER_CLOSE_POS);
        Action grabPreset = new Preset(
                Math.PI*60/180,
                16,
                Math.PI*180/180, WRIST_PERPEN_POS, FINGER_OPEN_POS);
//        drive.

        fingers.setPosition(FINGER_CLOSE_POS);
        wrist.setPosition(WRIST_START_POS);
        hand.setPosition(HAND_START_POS);

        waitForStart();

//        turret.setAngleRadians(Math.atan2(13, 8)+Math.PI*90/180);
//        slide.setLength(Math.sqrt(Math.pow(13, 2) + Math.pow(8, 2)));
//        hand.setPosition(handPosFromAngle(Math.PI*180/180, Math.atan2(13, 8)+Math.PI*90/180));
////        sleep(2000);
//        fl.setPower(0.75);
//        fr.setPower(0.75);
//        bl.setPower(0.75);
//        br.setPower(0.75);
//        sleep(2000);

        Actions.runBlocking(
                new SequentialAction(
                        highChamberPreset,
                        forwardAction));
//                        highChamberPreset,
//                        new Release()
//                )
//        );
    }
}
