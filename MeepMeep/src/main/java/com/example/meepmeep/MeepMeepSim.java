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

        RoadRunnerBotEntity redLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                .splineToSplineHeading(new Pose2d(-46.8, -37, Math.toRadians(270)), Math.toRadians(90))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, -57.0, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, -29, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, -59.2))
                                .back(12)

                                .build()
                );

        RoadRunnerBotEntity redCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                .splineToSplineHeading(new Pose2d(-36.2, -30, Math.toRadians(270)), Math.toRadians(90))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, -53, Math.toRadians(100)), Math.toRadians(315))
                                .splineToSplineHeading(new Pose2d(-2.3, -56, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, -36.5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, -59.2))
                                .back(12)
                                .build()
                );
        RoadRunnerBotEntity redRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                .splineToSplineHeading(new Pose2d(-30.3, -30.3, Math.toRadians(135)), Math.toRadians(0))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, -54, Math.toRadians(135)), Math.toRadians(315))
                                .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, -43.5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, -59.2))
                                .back(12)
                                .build()
                );

        RoadRunnerBotEntity rightRedRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                .splineToSplineHeading(new Pose2d(22.6, -37, Math.toRadians(270)), Math.toRadians(90))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))

                                .splineToSplineHeading(new Pose2d(47.7, -42, Math.toRadians(180)), Math.toRadians(30))
//
                                .strafeTo(new Vector2d(47.2, -59.2))
                                .back(12)

                                .build()
                );

        RoadRunnerBotEntity rightRedCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                        .back(1e-2)

                                        .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                        .splineToSplineHeading(new Pose2d(14.6, -33, Math.toRadians(270)), Math.toRadians(90))
                                        .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))

                                        .splineToSplineHeading(new Pose2d(47.7, -35, Math.toRadians(180)), Math.toRadians(30))
//
                                        .strafeTo(new Vector2d(47.2, -59.2))
                                        .back(12)

                                        .build()
                );

        RoadRunnerBotEntity rightRedLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                        .back(1e-2)

                                        .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
                                        .splineToSplineHeading(new Pose2d(7.8, -36, Math.toRadians(315)), Math.toRadians(180))
                                        .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))

                                        .splineToSplineHeading(new Pose2d(47.7, -29, Math.toRadians(180)), Math.toRadians(30))
//
                                        .strafeTo(new Vector2d(47.2, -59.2))
                                        .back(12)

                                        .build()
                );

        RoadRunnerBotEntity rightBlueLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-46.8, 37, Math.toRadians(90)), Math.toRadians(270))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, 57.0, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-2.3, 57, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, 29, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, 59.2))
                                .back(12)

                                .build()
                );
        RoadRunnerBotEntity rightBlueCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(270.0))
                                .splineToSplineHeading(new Pose2d(-36.2, 30, Math.toRadians(90)), Math.toRadians(270))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, 56, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-2.3, 56, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, 36.5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, 59.2))
                                .back(12)

                                .build()
                );
        RoadRunnerBotEntity rightBlueRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .back(1e-2)

                                .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(270.0))
                                .splineToSplineHeading(new Pose2d(-31.3, 34.3, Math.toRadians(135)), Math.toRadians(0))
                                .forward(5, veloConstraint(Math.toRadians(15), 5), accelConstraint(15))
                                //.splineToConstantHeading(new Vector2d(-40.6, -44.9), Math.toRadians(270)).setReversed(true)
                                .splineToSplineHeading(new Pose2d(-35.6, 57.0, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-2.3, 57, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(47.7, 43.5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(47.2, 59.2))
                                .back(12)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redCenterBot)
                .addEntity(redLeftBot)
                .addEntity(redRightBot)
                .addEntity(rightRedRightBot)
                .addEntity(rightRedCenterBot)
                .addEntity(rightRedLeftBot)
                .addEntity(rightBlueLeftBot)
                .addEntity(rightBlueCenterBot)
                .addEntity(rightBlueRightBot)
                .start();
    }
}