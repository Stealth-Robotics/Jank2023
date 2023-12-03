package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueCloseTrajectories {

    public static Trajectory leftDrop = TrajectoryBuilder.buildTrajectory(new Pose2d(16.5, 62, Math.toRadians(90)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16.5, 62, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(22.6, 43, Math.toRadians(90)), Math.toRadians(270))
            .build();

    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(leftDrop.end())
            .splineToConstantHeading(new Vector2d(26, 45), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, 42, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory centerDrop = TrajectoryBuilder.buildTrajectory(new Pose2d(16.5, 62, Math.toRadians(90)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16, 50, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(15, 27, Math.toRadians(90)), Math.toRadians(270))
            .splineToSplineHeading(new Pose2d(15, 32.75, Math.toRadians(90)), Math.toRadians(270),
                    SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(120),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory boardCenter = TrajectoryBuilder.buildTrajectory(centerDrop.end())
            .splineToConstantHeading(new Vector2d(26, 35), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, 35, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory rightDrop = TrajectoryBuilder.buildTrajectory(new Pose2d(16.5, 62, Math.toRadians(90)))
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(16.5, 58, Math.toRadians(45)), Math.toRadians(250))
            .splineToConstantHeading(new Vector2d(
                    5.5, 37), Math.toRadians(180))
            .build();
    public static Trajectory boardRight = TrajectoryBuilder.buildTrajectory(rightDrop.end())
            .forward(5)
            .splineToConstantHeading(new Vector2d(20, 32), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(42, 28, Math.toRadians(180)), Math.toRadians(0))
            .build();


    public static Trajectory parkRight = TrajectoryBuilder.buildTrajectory(boardRight.end())
            .strafeTo(new Vector2d(47.5, 57.6))
            .build();

}
