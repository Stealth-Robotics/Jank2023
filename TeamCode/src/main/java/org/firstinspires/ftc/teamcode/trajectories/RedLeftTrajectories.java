package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class RedLeftTrajectories {
    public static Trajectory example = TrajectoryBuilder.buildTrajectory(new Pose2d(0,0,0))
            .forward(5)
            .build();
    public static Trajectory slowExample = TrajectoryBuilder.buildTrajectory(example.end())
            .forward(5,
                    SampleMecanumDrive.getVelocityConstraint(15,
                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

    public static Trajectory leftPixelDrop = TrajectoryBuilder.buildTrajectory(
            new Pose2d(-39.5, -62, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-47, -33), Math.toRadians(90))
            .build();

    public static Trajectory outtakeDriveBack = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .back(
                    7,
                    SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(180),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(5)
            )
            .build();


}
