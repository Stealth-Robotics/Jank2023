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
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;
import java.util.Vector;

public class MeepMeepSim {
    private static TrajectoryVelocityConstraint veloConstraint(double angVel, double velo) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(angVel),
                new MecanumVelocityConstraint(velo, 10.113)
        ));
    }

    private static TrajectoryAccelerationConstraint accelConstraint(double constraint) {
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
                                .addTrajectory(RedLeftTrajectories.intakeFirstStackLeft)
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
                                .addTrajectory(RedLeftTrajectories.intakeFirstStackCenter)
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
                                .addTrajectory(RedLeftTrajectories.intakeFirstStackRight)

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
                                .addTrajectory(BlueRightTrajectories.parkLeft)
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
                                .addTrajectory(BlueRightTrajectories.parkCenter)
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
        Pose2d stackLocation = new Pose2d(-58, -41, Math.toRadians(140));
        Pose2d bluestackLocation = new Pose2d(-58, 42, Math.toRadians(220));
        Vector2d middleStackLocation = new Vector2d(-57.25, -37);
        Pose2d leftBoardScore = new Pose2d(47.7, -32, Math.toRadians(180));
        Pose2d blueRightBoardScore = new Pose2d(47.7, 32, Math.toRadians(180));

        RoadRunnerBotEntity rrpathgentest = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 150, Math.toRadians(200), Math.toRadians(200), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .back(1e-2)
                                .lineTo(new Vector2d(-43, -27))
                                .forward(1e-2)
                                .splineTo(new Vector2d(-50, -41), Math.toRadians(180))


                                .lineToSplineHeading(stackLocation)

                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-30, -58, Math.toRadians(180)), Math.toRadians(0))
                                .back(18)
                                .splineToSplineHeading(leftBoardScore, Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, -59.5, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineToSplineHeading(new Pose2d(-60.11, -40, Math.toRadians(140)), Math.toRadians(145))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-22.8, -59.5, Math.toRadians(180)), Math.toRadians(0))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(45.82, -48, Math.toRadians(180.00)), Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, -59.5, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineToSplineHeading(new Pose2d(-60.11, -40, Math.toRadians(140)), Math.toRadians(145))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-22.8, -59.5, Math.toRadians(180)), Math.toRadians(0))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(45.82, -48, Math.toRadians(180.00)), Math.toRadians(5.13))


                                .build()
                );

        RoadRunnerBotEntity middlePath = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 85, Math.toRadians(180), Math.toRadians(180), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, -62, Math.toRadians(270)))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-39.5, -60, Math.toRadians(270)), Math.toRadians(90.0))
                                .splineToSplineHeading(new Pose2d(-44.5, -36, Math.toRadians(270)), Math.toRadians(90))
                                .forward(3)
                                .splineTo(new Vector2d(-50, -42), Math.toRadians(0))


                                .lineToSplineHeading(stackLocation)
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-30, -38, Math.toRadians(180)), Math.toRadians(0))
                                .back(18)
                                .splineToSplineHeading(leftBoardScore, Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, -38, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineTo(middleStackLocation, Math.toRadians(180))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-22.8, -38, Math.toRadians(180)), Math.toRadians(0))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(45.82, -43, Math.toRadians(180.00)), Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, -38, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineTo(middleStackLocation, Math.toRadians(180))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-22.8, -38, Math.toRadians(180)), Math.toRadians(0))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(45.82, -43, Math.toRadians(180.00)), Math.toRadians(5.13))


                                .build()
                );


        RoadRunnerBotEntity blueFarAuto = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 150, Math.toRadians(200), Math.toRadians(200), 10.113)
                .setDimensions(13.75, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.5, 62, Math.toRadians(90)))

                                .lineTo(new Vector2d(-48, 34))


                                .forward(3)
                                .splineTo(new Vector2d(-48, 42), Math.toRadians(100))

                                .lineToSplineHeading(bluestackLocation)

                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-30, 60, Math.toRadians(180)), Math.toRadians(0))
                                .back(18)
                                .splineToSplineHeading(blueRightBoardScore, Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, 60, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineToSplineHeading(bluestackLocation, Math.toRadians(215))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-30, 60, Math.toRadians(180)), Math.toRadians(0))
                                .back(18)
                                .splineToSplineHeading(blueRightBoardScore, Math.toRadians(0))
                                .forward(1e-2)
                                .splineToSplineHeading(new Pose2d(-3, 60, Math.toRadians(180)), Math.toRadians(180))
                                .forward(1)
                                .splineToSplineHeading(bluestackLocation, Math.toRadians(215))
                                .back(1e-2)
                                .splineToSplineHeading(new Pose2d(-30, 60, Math.toRadians(180)), Math.toRadians(0))
                                .back(18)
                                .splineToSplineHeading(blueRightBoardScore, Math.toRadians(0))


                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(redCenterBot)
//                .addEntity(redLeftBot)
//                .addEntity(redRightBot)
//                .addEntity(rightRedRightBot)
//                .addEntity(blueRightBot)
//                .addEntity(blueCenterBot)
//                .addEntity(blueLeftBot)
//                .addEntity(rightRedCenterBot)
//                .addEntity(rightRedLeftBot)
//                .addEntity(rightBlueLeftBot)
//                .addEntity(rightBlueCenterBot)
//                .addEntity(rightBlueRightBot)
                .addEntity(rrpathgentest)
//                .addEntity(middlePath)
                .addEntity(blueFarAuto)
                .start();
    }
}