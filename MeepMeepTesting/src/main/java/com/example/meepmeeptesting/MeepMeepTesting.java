package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 60, Math.toRadians(90)))
//                .strafeTo(new Vector2d(50, 32))
//                .splineTo(new Vector2d(60, 60), Math.toRadians(90))
//
//                .strafeTo(new Vector2d(58, 32))
//                        .turn(Math.toRadians(45))
//                .strafeTo(new Vector2d(60, 60))
//                        .strafeTo(new Vector2d(66, 32))
//                        .strafeTo(new Vector2d(60, 60))
//
//// need to add more turns
//
//                .build());
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, 60, Math.toRadians(90)))
        .lineToY(24) // go to submersible, score first specimen
                .strafeTo(new Vector2d(-50, 32))
        .strafeTo(new Vector2d(-50, 60))
        .strafeTo(new Vector2d(-58, 32))
        .strafeTo(new Vector2d(-50, 60))
        .strafeTo(new Vector2d(-66, 32))
        .strafeTo(new Vector2d(-50, 60))
        .splineTo(new Vector2d(-12, 24), Math.toRadians(45))
        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

//                .setTangent(Math.toRadians(-15))
//        .lineToX(-50) // going to first sample
//                .setTangent(Math.toRadians(-90))
//        .lineToY(60) // going to drop off sample
//                .setTangent(Math.toRadians(78))
//        .lineToX(-56)
//                .setTangent(Math.toRadians(-90))
//        .lineToY(60)
//                .setTangent(Math.toRadians(78))
//        .lineToX(-62)
//                .setTangent(Math.toRadians(-90))
//        .splineToConstantHeading(new Vector2d(-48, 60), Math.toRadians(-60))
//        .splineToConstantHeading(new Vector2d(-12, 24), Math.toRadians(-45))
//        .splineToConstantHeading(new Vector2d(-48, 60), Math.toRadians(45))
//        .splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(-45))
//        .splineToConstantHeading(new Vector2d(-48, 60), Math.toRadians(45))
//        .splineToConstantHeading(new Vector2d(-8, 24), Math.toRadians(-45))
//        .splineToConstantHeading(new Vector2d(-48, 60), Math.toRadians(45))
//        .splineToConstantHeading(new Vector2d(-6, 24), Math.toRadians(-45))

//myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, 60, Math.toRadians(90)))
//        .lineToY(24) // go to submersible, score first specimen
//                .strafeTo(new Vector2d(-50, 32))
//        .strafeTo(new Vector2d(-50, 60))
//        .strafeTo(new Vector2d(-58, 32))
//        .strafeTo(new Vector2d(-50, 60))
//        .strafeTo(new Vector2d(-66, 32))
//        .strafeTo(new Vector2d(-50, 60))
//        .splineTo(new Vector2d(-12, 24), Math.toRadians(45))
//        .build());