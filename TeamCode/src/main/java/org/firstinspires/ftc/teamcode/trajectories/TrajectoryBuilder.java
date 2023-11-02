package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class TrajectoryBuilder {
    public static com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder buildTrajectory(com.acmerobotics.roadrunner.geometry.Pose2d startPose, boolean reversed){
        return new com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder(
                startPose,
                reversed,
                SampleMecanumDrive.VEL_CONSTRAINT,
                SampleMecanumDrive.ACCEL_CONSTRAINT
        );
    }
    public static com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder buildTrajectory(com.acmerobotics.roadrunner.geometry.Pose2d startPose){
        return buildTrajectory(startPose, false);
    }
}
