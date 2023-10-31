package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedLeftTrajectories {
    private static Trajectory example = TrajectoryBuilder.buildTrajectory(new Pose2d(0,0,0))
            .forward(5)
            .build();
    private static Trajectory slowExample = TrajectoryBuilder.buildTrajectory(example.end())
            .forward(5,
                    SampleMecanumDrive.getVelocityConstraint(15,
                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

    //first trajectory, drives to tape line
    public static Trajectory leftPixelDrop = TrajectoryBuilder.buildTrajectory(
            new Pose2d(-39.5, -62, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-47, -33), Math.toRadians(90))
            .build();
    public static Trajectory centerPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(-39.5, -62, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-40, -32.9), Math.toRadians(90))
            .build();
    public static Trajectory rightPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(-39.5, -62, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-39.5, -50), Math.toRadians(90))
            .splineTo(new Vector2d(-30.6, -34.9), Math.toRadians(45))
            .build();


    //next trajectory, drives back slowly to outtake hex
    public static Trajectory outtakeDriveBackLeft = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .back(
                    7,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackCenter = TrajectoryBuilder.buildTrajectory(centerPixelDrop.end())
            .back(
                    7,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackRight = TrajectoryBuilder.buildTrajectory(rightPixelDrop.end())
            .back(
                    3,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    //after outtaking hex, drive to board position to place hex
    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(outtakeDriveBackLeft.end())
            .splineTo(new Vector2d(-24.2, -36.7), Math.toRadians(0))
            .splineTo(new Vector2d(50, -28), Math.toRadians(0))
            .build();

    public static Trajectory driveToBoardCenter = TrajectoryBuilder.buildTrajectory(outtakeDriveBackCenter.end())
            .splineTo(new Vector2d(-24.2, -36.7), Math.toRadians(0))
            .splineTo(new Vector2d(50, -35), Math.toRadians(0))
            .build();
    public static TrajectorySequence driveToBoardRight = TrajectorySequenceBuilder.buildTrajectorySequence(outtakeDriveBackRight.end())
            .splineTo(new Vector2d(-34.1, -49.3), Math.toRadians(315))
            .splineTo(new Vector2d(-2.3, -57), Math.toRadians(0))
            .splineTo(new Vector2d(47.7, -43.5), Math.toRadians(0))
            .build();



}
