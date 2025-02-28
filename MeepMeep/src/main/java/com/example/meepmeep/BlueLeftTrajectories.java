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

public class BlueLeftTrajectories {

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
        return new TrajectoryBuilder(start, reversed,
                veloConstraint(Math.toRadians(120), 55),
                accelConstraint(50));
    }

    static Trajectory blueRightDrop = buildSequence(new Pose2d(-39.5, 62, Math.toRadians(90)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, 60, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(-46.8, 45, Math.toRadians(90)), Math.toRadians(270))
            .build();


    static Trajectory driveToBoardRight = buildSequence(blueRightDrop.end(), false)
            .splineToSplineHeading(new Pose2d(-29.8, 56.0, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, 57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, 29, Math.toRadians(180)), Math.toRadians(0))
            .build();

    static Trajectory redCenterDrop = buildSequence(new Pose2d(-39.5, 62, Math.toRadians(90)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(-36.2, 27, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(-36.2, 35, Math.toRadians(90)), Math.toRadians(90)            )
            .build();
    static Trajectory driveToBoardCenter = buildSequence(redCenterDrop.end(), false)

            //.splineToConstantHeading(new Vector2d(-34, -44), Math.toRadians(0))
            .forward(1e-2)
            .splineToConstantHeading(new Vector2d(-34, 58), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-25, 58, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-10, 58, Math.toRadians(180)), Math.toRadians(0))

            .splineToSplineHeading(new Pose2d(47.7, 34, Math.toRadians(180)), Math.toRadians(0))
            .build();

    static Trajectory blueLeftDrop = buildSequence(new Pose2d(-39.5, 62, Math.toRadians(90)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(0)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(-36, 28, Math.toRadians(20)), Math.toRadians(270))
            .build();
    static Trajectory driveToBoardLeft = buildSequence(blueLeftDrop.end(), true)
            .splineToConstantHeading(new Vector2d(-35.6, 56), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-20, 60, Math.toRadians(0)), Math.toRadians(350))
            .splineToSplineHeading(new Pose2d(-2.3, 60, Math.toRadians(0)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, 41, Math.toRadians(180)), Math.toRadians(0))
            .build();
}
