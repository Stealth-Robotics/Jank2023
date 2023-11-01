package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedRightTrajectories {

    public static Trajectory leftPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(90.00)))
            .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(7.8, -36, Math.toRadians(135)), Math.toRadians(180))
            .build();
    public static Trajectory centerPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(90.00)))
            .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(14.6, -33, Math.toRadians(90)), Math.toRadians(90))
            .build();
    public static Trajectory rightPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(16.5, -62, Math.toRadians(90.00)))
            .splineToSplineHeading(new Pose2d(16.5, -50, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(22.6, -37, Math.toRadians(90)), Math.toRadians(90))
            .build();


    //next trajectory, drives back slowly to outtake hex
    public static Trajectory outtakeDriveBackLeft = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .back(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackCenter = TrajectoryBuilder.buildTrajectory(centerPixelDrop.end())
            .back(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackRight = TrajectoryBuilder.buildTrajectory(rightPixelDrop.end())
            .back(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    //after outtaking hex, drive to board position to place hex
    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(outtakeDriveBackLeft.end())
            .splineToSplineHeading(new Pose2d(47.7, -29, Math.toRadians(180)), Math.toRadians(30))
            .build();

    public static Trajectory driveToBoardCenter = TrajectoryBuilder.buildTrajectory(outtakeDriveBackCenter.end())
            .splineToSplineHeading(new Pose2d(47.7, -35, Math.toRadians(180)), Math.toRadians(30))

            .build();
    public static TrajectorySequence driveToBoardRight = TrajectorySequenceBuilder.buildTrajectorySequence(outtakeDriveBackRight.end())
            .splineToSplineHeading(new Pose2d(47.7, -42, Math.toRadians(180)), Math.toRadians(30))
            .build();

}
