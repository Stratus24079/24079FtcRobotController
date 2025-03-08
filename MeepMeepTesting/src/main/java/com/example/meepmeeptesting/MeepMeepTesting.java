package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(5, -30), Math.toRadians(270))
                .strafeTo(new Vector2d(5, -35))

                .strafeToSplineHeading(new Vector2d(25, -45), Math.toRadians(90)) // To 1st sample
                .strafeToSplineHeading(new Vector2d(44, 0), Math.toRadians(90))

                .splineToSplineHeading(new Pose2d(44, -55, Math.toRadians(90)), Math.toRadians(270)) // To human player

                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(54, 0, Math.toRadians(90)), Math.toRadians(45)) // To 2nd sample
                .splineToSplineHeading(new Pose2d(54, -55, Math.toRadians(90)), Math.toRadians(270)) // To human player

                .splineToSplineHeading(new Pose2d(62, 0, Math.toRadians(90)), Math.toRadians(270)) // To 3rd sample
                .splineToSplineHeading(new Pose2d(62, -55, Math.toRadians(90)), Math.toRadians(270)) // To human player

                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(90)) // Get 1st specimen
                .strafeTo(new Vector2d(40, -60))
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .strafeTo(new Vector2d(10, -35))

                .strafeToLinearHeading(new Vector2d(40, -45), Math.toRadians(90)) // Get 2nd specimen
                .strafeTo(new Vector2d(40, -50))
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .strafeTo(new Vector2d(10, -35))

                .strafeToLinearHeading(new Vector2d(40, -45), Math.toRadians(90)) // Get 3rd specimen
                .strafeTo(new Vector2d(40, -50))
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .strafeTo(new Vector2d(10, -35))
                .strafeTo(new Vector2d(30, -50))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}