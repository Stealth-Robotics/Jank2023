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
        return new TrajectoryBuilder(start, reversed,
                veloConstraint(Math.toRadians(100), 45),
                accelConstraint(30));
    }

    static Trajectory redLeftLeftDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, -60, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-46.8, -25, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-46.8, -32, Math.toRadians(270)), Math.toRadians(270))
            .build();


    public static Trajectory intakeFirstStackLeft = buildSequence(redLeftLeftDrop.end(), false)
            .forward(2)
//            .splineToSplineHeading(new Pose2d(-52.5, -35, Math.toRadians(180)), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-55, -35, Math.toRadians(180)), Math.toRadians(180))

            .build();




    static Trajectory driveToBoardLeft = buildSequence(intakeFirstStackLeft.end(), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-43, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-24, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -29, Math.toRadians(180)), Math.toRadians(0))
            .build();

    static Trajectory redCenterDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-36.2, -27, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36.2, -35, Math.toRadians(270)), Math.toRadians(270)            )
            .build();

    public static Trajectory intakeFirstStackCenter = buildSequence(redCenterDrop.end(), false)
            .forward(2)
//            .splineToSplineHeading(new Pose2d(-52.5, -35, Math.toRadians(180)), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-55, -35, Math.toRadians(180)), Math.toRadians(180))

            .build();


    static Trajectory driveToBoardCenter = buildSequence(intakeFirstStackCenter.end(), false)

            //.splineToConstantHeading(new Vector2d(-34, -44), Math.toRadians(0))
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-43, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-24, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -35, Math.toRadians(180)), Math.toRadians(0))
            .build();

    static Trajectory redRightDrop = buildSequence(new Pose2d(-39.5, -62, Math.toRadians(270)), false)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-34, -22, Math.toRadians(160)), Math.toRadians(90))
            .build();
    public static Trajectory intakeFirstStackRight = buildSequence(redRightDrop.end(), false)
            .forward(2)
//            .splineToSplineHeading(new Pose2d(-52.5, -35, Math.toRadians(180)), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-55, -35, Math.toRadians(180)), Math.toRadians(180))

            .build();
    static Trajectory driveToBoardRight = buildSequence(intakeFirstStackRight.end(), true)
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-43, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-24, -35, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -40, Math.toRadians(180)), Math.toRadians(0))
            .build();
}
