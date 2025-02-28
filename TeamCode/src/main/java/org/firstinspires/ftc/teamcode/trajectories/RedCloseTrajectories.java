package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedCloseTrajectories {

    public static Trajectory leftPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(270)))
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(14, -50, Math.toRadians(315)), Math.toRadians(110))
            .splineToConstantHeading(new Vector2d(
                    5, -38), Math.toRadians(180))
            .build();
    public static Trajectory centerPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(270)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16, -50, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(16, -27, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(16, -35, Math.toRadians(270)), Math.toRadians(270)            )
            .build();
    public static Trajectory rightPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(270)), Math.toRadians(270))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(16.5, -60, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(22.6, -45, Math.toRadians(270)), Math.toRadians(90))
            .build();

    //after outtaking hex, drive to board position to place hex
    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .strafeLeft(1e-2)
            .splineToConstantHeading(new Vector2d(15, -32), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(42, -26.5, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory driveToBoardCenter = TrajectoryBuilder.buildTrajectory(centerPixelDrop.end())
            .strafeLeft(1e-2)
            .splineToConstantHeading(new Vector2d(26, -35), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(0))
            .build();
    public static Trajectory driveToBoardRight = TrajectoryBuilder.buildTrajectory(rightPixelDrop.end())
            .strafeLeft(1e-2)
            .splineToConstantHeading(new Vector2d(26, -45), Math.toRadians(-1e-10))
            .splineToSplineHeading(new Pose2d(42, -42, Math.toRadians(180)), Math.toRadians(0))
            .build();

}
