package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    private static TrajectoryVelocityConstraint veloConstraint(double angVel, double velo){
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(angVel),
                new MecanumVelocityConstraint(velo, 15.5)
        ));
    }
    private static TrajectoryAccelerationConstraint accelConstraint(double constraint){
        return new ProfileAccelerationConstraint(constraint);
    }
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(80, 50, Math.toRadians(180), Math.toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(62.17, -34.18, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(32.05, -24), Math.toRadians(185.67))
                                .back(7, veloConstraint(Math.toRadians(180), 5), accelConstraint(5))
                                .splineToLinearHeading(new Pose2d(34.3, -24, Math.toRadians(270)),
                                        Math.toRadians(270),
                                        veloConstraint(Math.toRadians(60), 50),
                                        accelConstraint(40)
                                )

                                .lineTo(
                                        new Vector2d(37.82, 52.99),
                                        veloConstraint(2, 75),
                                        accelConstraint(50)
                                )
                                .build()


                );

        Image img = null;
        try{img = ImageIO.read(new File("C:/users/oliver/Downloads/field.png"));}
        catch (IOException e){}

        mm.setBackground(img)
            .setBackgroundAlpha(0.5f)
            .addEntity(myBot)
            .start();

    }
}