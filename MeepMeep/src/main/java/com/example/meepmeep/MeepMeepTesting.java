package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 0)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.6, -62, Math.toRadians(90)))
                                /*.forward(12)
                                .waitSeconds(0.5)
                                .strafeLeft(8)
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(-11.8, -42))
                                .lineToLinearHeading(new Pose2d(10, -63, Math.toRadians(0)))
                                .lineTo(new Vector2d(57, -63))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(40, -63, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-11.8, -42, Math.toRadians(90)), Math.toRadians(90)) */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}