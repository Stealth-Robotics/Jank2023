package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(60, 30, Math.toRadians(180), Math.toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(62.17, -34.18, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(32.05, -37.18), Math.toRadians(185.67))
                                .splineTo(new Vector2d(39.31, -64.95), Math.toRadians(-75.34))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(37.82, 52.99))
                                .build()


                );

        Image img = null;
        try{img = ImageIO.read(new File("C:/users/oliver/Downloads/image.png"));}
        catch (IOException e){}

        mm.setBackground(img)
            .setBackgroundAlpha(1f)
            .addEntity(myBot)
            .start();

    }
}