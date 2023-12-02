package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

public class MeepMeepSim{
    private static TrajectoryVelocityConstraint veloConstraint(double angVel, double velo){
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(angVel),
                new MecanumVelocityConstraint(velo, 10.113)
        ));
    }
    private static TrajectoryAccelerationConstraint accelConstraint(double constraint){
        return new ProfileAccelerationConstraint(constraint);
    }



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity redLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .addTrajectory(RedLeftTrajectories.redLeftLeftDrop)

                                //.addTrajectory(RedLeftTrajectories.leftDriveForward)
                                //.waitSeconds(0.5)
                                .addTrajectory(RedLeftTrajectories.driveToBoardLeft)
                                .build()
                );

        RoadRunnerBotEntity redCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(100), Math.toRadians(100), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .addTrajectory(RedLeftTrajectories.redCenterDrop)
                                .addTrajectory(RedLeftTrajectories.driveToBoardCenter)
                                .build()
                );
        RoadRunnerBotEntity redRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .addTrajectory(RedLeftTrajectories.redRightDrop)
                                .addTrajectory(RedLeftTrajectories.driveToBoardRight)
                                .build()
                );

        RoadRunnerBotEntity rightRedRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                .addTrajectory(RedRightTrajectories.redRightRightDrop)
                                .addTrajectory(RedRightTrajectories.driveToBoardRight)
                                .build()
                );

        RoadRunnerBotEntity rightRedCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                        .addTrajectory(RedRightTrajectories.centerDrop)
                                        .addTrajectory(RedRightTrajectories.boardCenter)
                                        .build()
                );

        RoadRunnerBotEntity rightRedLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16.5, -62, Math.toRadians(270)))
                                        .addTrajectory(RedRightTrajectories.leftDrop)
                                        .addTrajectory(RedRightTrajectories.boardLeft)
                                        .build()
                );

        RoadRunnerBotEntity blueRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .addTrajectory(BlueLeftTrajectories.blueRightDrop)

                                //.addTrajectory(RedLeftTrajectories.leftDriveForward)
                                //.waitSeconds(0.5)
                                .addTrajectory(BlueLeftTrajectories.driveToBoardRight)

                                .build()
                );

        RoadRunnerBotEntity blueCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(100), Math.toRadians(100), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .addTrajectory(BlueLeftTrajectories.redCenterDrop)
                                .addTrajectory(BlueLeftTrajectories.driveToBoardCenter)
                                .build()
                );
        RoadRunnerBotEntity blueLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(90)))
                                .addTrajectory(BlueLeftTrajectories.blueLeftDrop)
                                .addTrajectory(BlueLeftTrajectories.driveToBoardLeft)
                                .build()
                );


        RoadRunnerBotEntity rightBlueLeftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .addTrajectory(BlueRightTrajectories.blueRightLeftDrop)
                                .addTrajectory(BlueRightTrajectories.driveToBoardLeft)
                                .build()
                );
        RoadRunnerBotEntity rightBlueCenterBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .addTrajectory(BlueRightTrajectories.centerDrop)
                                .addTrajectory(BlueRightTrajectories.boardCenter)
                                .build()
                );

        RoadRunnerBotEntity rightBlueRightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(250), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))
                                .addTrajectory(BlueRightTrajectories.rightDrop)
                                .addTrajectory(BlueRightTrajectories.boardRight)
                                .addTrajectory(BlueRightTrajectories.parkRight)
                                .build()
                );





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redCenterBot)
                .addEntity(redLeftBot)
                .addEntity(redRightBot)
                .addEntity(rightRedRightBot)
                .addEntity(blueRightBot)
                .addEntity(blueCenterBot)
                .addEntity(blueLeftBot)
                .addEntity(rightRedCenterBot)
                .addEntity(rightRedLeftBot)
                .addEntity(rightBlueLeftBot)
                .addEntity(rightBlueCenterBot)
                .addEntity(rightBlueRightBot)
                .start();
    }
}