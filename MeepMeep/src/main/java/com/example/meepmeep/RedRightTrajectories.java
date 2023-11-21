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

public class RedRightTrajectories {

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


    public static Trajectory redRightRightDrop = buildSequence(new Pose2d(16.5, -62, Math.toRadians(270)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16.5, -60, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(22.6, -45, Math.toRadians(270)), Math.toRadians(90))
            .build();

    public static Trajectory driveToBoardRight = buildSequence(redRightRightDrop.end(), false)
            .splineToConstantHeading(new Vector2d(26, -45), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, -42, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory centerDrop = buildSequence(new Pose2d(16.5, -62, Math.toRadians(270)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16, -50, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(16, -27, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(16, -35, Math.toRadians(270)), Math.toRadians(270)            )
            .build();
    public static Trajectory boardCenter = buildSequence(centerDrop.end(), false)
            .splineToConstantHeading(new Vector2d(26, -35), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory leftDrop = buildSequence(new Pose2d(16.5, -62, Math.toRadians(270)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(14, -45, Math.toRadians(315)), Math.toRadians(110))
            .splineToConstantHeading(new Vector2d(
                    5, -38), Math.toRadians(180))
            .build();
    public static Trajectory boardLeft = buildSequence(leftDrop.end(), false)
            .splineToConstantHeading(new Vector2d(10, -32), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(42, -30, Math.toRadians(180)), Math.toRadians(0))
            .build();


    public static Trajectory parkRight = buildSequence(driveToBoardRight.end(), false)
            .strafeTo(new Vector2d(47.5, -57.6))
            .build();

}
