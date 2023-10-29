package com.example.meepmeep;

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

import java.util.Arrays;

public class MeepMeepSim{
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
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-39.5, -42), Math.toRadians(90))
                                .splineTo(new Vector2d(-24.9, -32.9), Math.toRadians(90))
                                .back(3, veloConstraint(Math.toRadians(180), 5), accelConstraint(5))
                                .forward(1.5)
                                .turn(Math.toRadians(90))

                                .back(3)
                                .splineTo(new Vector2d(2, -35.3), Math.toRadians(0))
                                .back(1)
                                .splineTo(new Vector2d(50, -41), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}