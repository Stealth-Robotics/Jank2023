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
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

public class BlueRightTrajectories {

    private static TrajectoryVelocityConstraint veloConstraint(double angVel, double velo){
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(angVel),
                new MecanumVelocityConstraint(velo, 10.113)
        ));
    }
    private static TrajectoryAccelerationConstraint accelConstraint(double constraint){
        return new ProfileAccelerationConstraint(constraint);
    }


    private static TrajectoryBuilder buildSequence(Pose2d start, double heading){
        return new TrajectoryBuilder(start, heading,
                veloConstraint(Math.toRadians(120), 55),
                accelConstraint(50));
    }
    private static TrajectoryBuilder buildSequence(Pose2d start, boolean reversed){
        return new TrajectoryBuilder(start, reversed,
                veloConstraint(Math.toRadians(100), 45),
                accelConstraint(30));
    }

    private static TrajectorySequenceBuilder buildTrajSequence(Pose2d start){
        return new TrajectorySequenceBuilder(start,
                veloConstraint(Math.toRadians(100), 45),
                accelConstraint(30), 100, 100)
                ;
    }

    public static Trajectory blueRightLeftDrop = buildSequence(new Pose2d(16.5, 62, Math.toRadians(90)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16.5, 60, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(22.6, 45, Math.toRadians(90)), Math.toRadians(270))
            .build();

    public static Trajectory driveToBoardLeft = buildSequence(blueRightLeftDrop.end(), false)
            .splineToConstantHeading(new Vector2d(26, 45), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, 42, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory centerDrop = buildSequence(new Pose2d(16.5, 62, Math.toRadians(90)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16, 50, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(16, 27, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(16, 35, Math.toRadians(90)), Math.toRadians(270)            )
            .build();
    public static Trajectory boardCenter = buildSequence(centerDrop.end(), false)
            .splineToConstantHeading(new Vector2d(26, 35), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, 35, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory rightDrop = buildSequence(new Pose2d(16.5, 62, Math.toRadians(90)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(14, 45, Math.toRadians(45)), Math.toRadians(250))
            .splineToConstantHeading(new Vector2d(
                    6, 38), Math.toRadians(180))
            .build();
    public static Trajectory boardRight = buildSequence(rightDrop.end(), false)
            .forward(5)
            .splineToConstantHeading(new Vector2d(20, 32), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(42, 30, Math.toRadians(180)), Math.toRadians(0))
            .build();


    public static Trajectory parkRight = buildSequence(boardRight.end(), false)
            .strafeLeft(18)

            .build();

    public static Trajectory parkLeft = buildSequence(driveToBoardLeft.end(), false)
            .strafeLeft(30)
            .build();

    public static Trajectory parkCenter = buildSequence(boardCenter.end(), false)
            .strafeLeft(25)
            .build();

}


