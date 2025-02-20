package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        TrajectoryActionBuilder forward = myBot.getDrive().actionBuilder(new Pose2d(9, 63, Math.toRadians(-90))).lineToY(AutoTunables.SPECIMEN_FORWARD, new TranslationalVelConstraint(AutoTunables.SPECIMEN_FORWARD_SPEED*50));
        TrajectoryActionBuilder moveToSamples = forward.endTrajectory().fresh().
                setTangent(Math.toRadians(0)).splineToConstantHeading(new Vector2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_Y), 0);
        TrajectoryActionBuilder moveToBasket = moveToSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(45));
        TrajectoryActionBuilder goToSample2FromBasket = moveToBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(AutoTunables.SAMPLE_X + 10.5, AutoTunables.SAMPLE_Y, Math.toRadians(-90)), Math.toRadians(-90));
        TrajectoryActionBuilder moveToBasket2 = goToSample2FromBasket.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(90));
        TrajectoryActionBuilder goToSample3FromBasketPart1 = moveToBasket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.SAMPLE_X, AutoTunables.SAMPLE_3_Y, Math.toRadians(0)), Math.toRadians(0))
                .lineToX(AutoTunables.SAMPLE_3_X);;
        TrajectoryActionBuilder moveToBasket3 = goToSample3FromBasketPart1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-180))
                .lineToX(AutoTunables.SAMPLE_X-3)
                .splineToSplineHeading(new Pose2d(AutoTunables.BASKET_X, AutoTunables.BASKET_Y, Math.toRadians(45)), Math.toRadians(45));
        TrajectoryActionBuilder end = moveToBasket3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(AutoTunables.SAMPLE_X-8, AutoTunables.END_Y, Math.toRadians(180)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(AutoTunables.END_X, AutoTunables.END_Y, Math.toRadians(-180)), Math.toRadians(-180));

        myBot.runAction(new SequentialAction(
                //Score preload specimen
                forward.build(),
                moveToSamples.build(),
                moveToBasket.build(),
                goToSample2FromBasket.build(),
                moveToBasket2.build(),
                goToSample3FromBasketPart1.build(),
                moveToBasket3.build(),
                end.build()
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}