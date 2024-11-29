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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14.5, -56, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(6, -36))
//                .turnTo(Math.toRadians(-28))
//                .strafeToLinearHeading(new Vector2d(36, -42), Math.toRadians(35))
//                .turnTo(Math.toRadians(-30))
//                .strafeToLinearHeading(new Vector2d(44, -42), Math.toRadians(35))
//                .turnTo(Math.toRadians(-32))
//                .strafeToLinearHeading(new Vector2d(38, -42), Math.toRadians(270))
//                .setTangent(Math.toRadians(270))
//                .lineToYConstantHeading(-54)
//                .strafeToConstantHeading(new Vector2d(-4, -34))
//                .strafeToLinearHeading(new Vector2d(38, -42), Math.toRadians(270))
//                .setTangent(Math.toRadians(270))
//                .lineToYConstantHeading(-54)
//                .strafeToConstantHeading(new Vector2d(-4, -34))
//                .strafeToLinearHeading(new Vector2d(38, -42), Math.toRadians(270))
//                .setTangent(Math.toRadians(270))
//                .lineToYConstantHeading(-54)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}