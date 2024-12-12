package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Slide;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Autonomous
public class AutoRedLeft extends LinearOpMode {
    MecanumDrive drive;
    Slide slide;
    Turret turret;
    Servo hand, wrist, fingers;
    final double FINGER_CLOSE_POS = 0.48;
    final double FINGER_OPEN_POS = 0.75;
    final double HAND_START_POS = 1.0;
    final double HAND_PARALLEL_POS = 0.365; //Pos where hand is parallel with slides
    final double WRIST_START_POS = 0.9;
    final double WRIST_PERPEN_POS = 0.47; //Pos where wrist is perpendicular to slides
    public double handPosFromAngle(double angle, double turretAngle){
        return (angle-(Math.PI/2-turretAngle)+Math.PI*0/180)*(HAND_PARALLEL_POS-HAND_START_POS)/(Math.PI*180/180)+HAND_START_POS;
    }

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(-8.5, -72+8.5, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        slide = new Slide(hardwareMap.get(DcMotorEx.class, "slideLeft"));
        turret = new Turret(hardwareMap.get(DcMotorEx.class, "turretLeft"));
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        fingers = hardwareMap.get(Servo.class, "fingers");

        Action forward = drive.actionBuilder(initialPose)
                        .lineToY(-24*2+8.5+5).build();

        fingers.setPosition(FINGER_CLOSE_POS);
        wrist.setPosition(WRIST_START_POS);

        waitForStart();

        turret.setAngleRadians(Math.atan2(13, 8)+Math.PI*90/180);
                slide.setLength(Math.sqrt(Math.pow(13, 2) + Math.pow(8, 2)));
                hand.setPosition(handPosFromAngle(Math.PI*180/180, Math.atan2(13, 8)+Math.PI*90/180));
                sleep(2000);

        Actions.runBlocking(
                new SequentialAction(
                        forward
                )
        );
    }
}
