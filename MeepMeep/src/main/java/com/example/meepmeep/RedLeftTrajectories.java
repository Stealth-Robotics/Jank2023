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

import java.util.Arrays;

public class RedLeftTrajectories {

    private static TrajectoryVelocityConstraint veloConstraint(double angVel, double velo){
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(angVel),
                new MecanumVelocityConstraint(velo, 10.113)
        ));
    }
    private static TrajectoryAccelerationConstraint accelConstraint(double constraint){
        return new ProfileAccelerationConstraint(constraint);
    }

    private static TrajectoryBuilder buildSequence(Pose2d start, boolean reversed){
        return new TrajectoryBuilder(start, reversed, veloConstraint(250, 40), accelConstraint(40));
    }

    static Trajectory redLeftLeftDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, -60, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-46.8, -45, Math.toRadians(270)), Math.toRadians(90))
            .build();


    static Trajectory driveToBoardLeft = buildSequence(redLeftLeftDrop.end(), false)
            .splineToSplineHeading(new Pose2d(-29.8, -56.0, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -29, Math.toRadians(180)), Math.toRadians(0))
            .build();

    static Trajectory redCenterDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-36.2, -27, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36.2, -35, Math.toRadians(270)), Math.toRadians(270),
                    veloConstraint(10, 4),
                    accelConstraint(4)
            )
            .build();
    static Trajectory driveToBoardCenter = buildSequence(redCenterDrop.end(), false)

            .splineToConstantHeading(new Vector2d(-31, -56), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(10, -56, Math.toRadians(270)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -34, Math.toRadians(180)), Math.toRadians(0))
            .build();


    static Trajectory redRightDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(180)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(160)), Math.toRadians(90))
            .build();
    static Trajectory driveToBoardRight = buildSequence(redRightDrop.end(), true)
            .splineToConstantHeading(new Vector2d(-35.6, -56), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -41, Math.toRadians(180)), Math.toRadians(0))
            .build();
}
