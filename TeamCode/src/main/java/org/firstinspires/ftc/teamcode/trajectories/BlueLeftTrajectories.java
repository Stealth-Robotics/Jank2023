package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueLeftTrajectories {
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
    public static TrajectorySequence leftPixelDrop = TrajectorySequenceBuilder.buildTrajectorySequence(
                    new Pose2d(-39.5, 62, Math.toRadians(90)))
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, 60, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-46.8, 45, Math.toRadians(90)), Math.toRadians(90))
            .build();
    public static TrajectorySequence centerPixelDrop = TrajectorySequenceBuilder.buildTrajectorySequence(
                    new Pose2d(-39.5, 62, Math.toRadians(90)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-36.2, 27, Math.toRadians(90)), Math.toRadians(90))
            .waitSeconds(0.5)
            .forward(9)
            .build();
    public static TrajectorySequence rightPixelDrop = TrajectorySequenceBuilder.buildTrajectorySequence(
                    new Pose2d(-39.5, 62, Math.toRadians(90)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, 50, Math.toRadians(90)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-31, 28, Math.toRadians(160)), Math.toRadians(0))
            .build();


    //next trajectory, drives back slowly to outtake hex
    public static Trajectory outtakeDriveBackLeft = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .forward(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackCenter = TrajectoryBuilder.buildTrajectory(centerPixelDrop.end())
            .forward(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    public static Trajectory outtakeDriveBackRight = TrajectoryBuilder.buildTrajectory(rightPixelDrop.end())
            .forward(
                    5,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();
    //after outtaking hex, drive to board position to place hex
    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(outtakeDriveBackLeft.end(), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-35.6, 57.0, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, 57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, 29, Math.toRadians(180)), Math.toRadians(0))
            .build();

    public static Trajectory driveToBoardCenter = TrajectoryBuilder.buildTrajectory(outtakeDriveBackCenter.end(), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-35.6, 50, Math.toRadians(100)), Math.toRadians(315))
            .splineToSplineHeading(new Pose2d(-2.3, 56, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, 34, Math.toRadians(180)), Math.toRadians(0))
            .build();
    public static Trajectory driveToBoardRight = TrajectoryBuilder.buildTrajectory(outtakeDriveBackRight.end(), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-35.6, 50, Math.toRadians(135)), Math.toRadians(315))
            .splineToSplineHeading(new Pose2d(-2.3, 57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, 41, Math.toRadians(180)), Math.toRadians(0))
            .build();


}
