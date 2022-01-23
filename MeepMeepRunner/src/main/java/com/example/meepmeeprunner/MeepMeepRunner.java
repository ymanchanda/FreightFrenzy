package com.example.meepmeeprunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRunner {
    public static void main(String args[]){
        MeepMeep mm = new MeepMeep(800);
        Pose2d startPoseAlmond = new Pose2d();
        Pose2d startPose = new Pose2d(8.5,-63, Math.toRadians(90));
/*
        RoadRunnerBotEntity AlmondBot = new DefaultBotBuilder(mm)
                .setConstraints()
                .followTrajectorySequence(driveA ->
                        driveA.trajectorySequenceBuilder(startPoseAlmond)
                            .build()
                );
*/
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(36.58665032249647,36.58665032249647,Math.toRadians(185.83871010638296),Math.toRadians(185.83871010638296),14.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
//                                .splineToLinearHeading(new Pose2d(-54.5,-60,Math.toRadians(180)), Math.toRadians(-180))
//                                //spin the carousel
//                                .waitSeconds(3)
//                                .lineTo(new Vector2d(-12,-40))
//                                //drop preloaded stone
//                                .waitSeconds(1)
//                                .splineToConstantHeading(new Vector2d(-28,-60),Math.toRadians(-180))
//                                .splineToLinearHeading(new Pose2d(-55,-40, Math.toRadians(0)),Math.toRadians(-180))
                                .splineTo(new Vector2d(6,-24), Math.toRadians(90))
                                .waitSeconds(1)
                                //.strafeTo(new Vector2d(7,-24))
                                .splineToLinearHeading(new Pose2d(-6, -55, Math.toRadians(0)), Math.toRadians(180))
                                .lineTo(new Vector2d(0,-66))
                                .lineTo(new Vector2d(49,-66))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(AlmondBot)
                .start();
    }
}