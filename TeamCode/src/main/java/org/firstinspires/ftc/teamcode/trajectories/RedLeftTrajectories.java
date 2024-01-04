package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

public class RedLeftTrajectories {
    public enum Position{
        RIGHT,
        LEFT
    }
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

    private static Pose2d stackLocation = new Pose2d(-60, -40, Math.toRadians(140));

    private static Vector2d middleStackLocation = new Vector2d(-55, -37);

    private static Pose2d leftBoardScore = new Pose2d(50, -32, Math.toRadians(180));

    private static Pose2d centerBoardScore;

    private static Pose2d rightBoardScore = new Pose2d(50, -43, Math.toRadians(180));



    //first trajectory, drives to tape line
    public static Trajectory leftPixelDrop = TrajectoryBuilder.buildTrajectory(

            new Pose2d(-39.5, -62, Math.toRadians(270)))

            .lineTo(new Vector2d(-44.5, -36))
            .build();

    //trajectory to pick up first hex from stack
    public static TrajectorySequence leftFirstStackIntake = TrajectorySequenceBuilder.buildTrajectory(leftPixelDrop.end())
            .forward(3)
            .splineTo(new Vector2d(-50, -40), Math.toRadians(180))

            .lineToSplineHeading(stackLocation)

            .build();

    public static TrajectorySequence groundPickup = TrajectorySequenceBuilder.buildTrajectory(leftFirstStackIntake.end())
            .back(5)
            .forward(2)
            .build();



    //drives to board to drop yellow and white corresponding to marker location
    public static Trajectory yellowAndWhiteBoardDropLeft = TrajectoryBuilder.buildTrajectory(leftFirstStackIntake.end())
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-22.8, -56, Math.toRadians(180)), Math.toRadians(0))
            .back(15)
            .splineToSplineHeading(leftBoardScore, Math.toRadians(0))
            .build();

    public static Trajectory yellowAndWhiteBoardDropLeftSpicyPath = TrajectoryBuilder.buildTrajectory(groundPickup.end())
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-30, -38, Math.toRadians(180)), Math.toRadians(0))
            .back(18)
            .splineToSplineHeading(leftBoardScore, Math.toRadians(0))
            .build();

    //drives back to stack to intake next two hexes
    public static Trajectory dropTwoWhites(Position side){
        Pose2d endPose = new Pose2d();
        if(side == Position.LEFT){
            endPose = leftBoardScore;
        }
        else if(side == Position.RIGHT){
            endPose = rightBoardScore;
        }
        return TrajectoryBuilder.buildTrajectory(stackLocation)
                .back(1e-2)
                .splineToSplineHeading(new Pose2d(-22.8, -58, Math.toRadians(180)), Math.toRadians(0))
                .back(10)
                .splineToSplineHeading(endPose, Math.toRadians(0))
                .build();
    }

    public static Trajectory dropTwoWhitesSpicyPath(Position side){
        Pose2d endPose = new Pose2d();
        if(side == Position.LEFT){
            endPose = leftBoardScore;
        }
        else if(side == Position.RIGHT){
            endPose = rightBoardScore;
        }
        return TrajectoryBuilder.buildTrajectory(new Pose2d(middleStackLocation.getX(), middleStackLocation.getY(),
                        Math.toRadians(180)))
                .back(1e-2)
                .splineToSplineHeading(new Pose2d(-22.8, -38, Math.toRadians(180)), Math.toRadians(0))
                .back(10)
                .splineToSplineHeading(endPose, Math.toRadians(0))
                .build();
    }

    public static Trajectory driveToStack(Position start){
        Pose2d startPose = new Pose2d();
        if(start == Position.LEFT){
            startPose = leftBoardScore;
        }
        else if(start == Position.RIGHT){
            startPose = rightBoardScore;
        }
        return TrajectoryBuilder.buildTrajectory(startPose)
                .forward(1e-2)
                .splineToSplineHeading(new Pose2d(-3, -62, Math.toRadians(180)), Math.toRadians(180))
                .forward(1)
                .splineToSplineHeading(stackLocation, Math.toRadians(145))
                .build();

    }
    public static Trajectory driveToStack(Position start, double yOffset){
        Pose2d startPose = new Pose2d();
        if(start == Position.LEFT){
            startPose = leftBoardScore;
        }
        else if(start == Position.RIGHT){
            startPose = rightBoardScore;
        }
        return TrajectoryBuilder.buildTrajectory(startPose)
                .forward(1e-2)
                .splineToSplineHeading(new Pose2d(-3, -58 - yOffset, Math.toRadians(180)), Math.toRadians(180))
                .forward(1)
                .splineToSplineHeading(new Pose2d(stackLocation.getX(), stackLocation.getY() - yOffset, stackLocation.getHeading()), Math.toRadians(145))
                .build();

    }

    public static Trajectory driveToStackSpicyPath(Position start, double yOffset){
        Pose2d startPose = new Pose2d();
        if(start == Position.LEFT){
            startPose = leftBoardScore;
        }
        else if(start == Position.RIGHT){
            startPose = rightBoardScore;
        }
        return TrajectoryBuilder.buildTrajectory(startPose)
                .forward(1e-2)
                .splineToSplineHeading(new Pose2d(-3, -38, Math.toRadians(180)), Math.toRadians(180))
                .forward(1)
                .splineTo(new Vector2d(middleStackLocation.getX(), middleStackLocation.getY() - yOffset), Math.toRadians(180))
                .build();

    }

    public static Trajectory centerPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(-39.5, -62, Math.toRadians(270)))
            .back(1e-2)

            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(270)), Math.toRadians(90.0))
            .splineToSplineHeading(new Pose2d(-36.2, -27, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36.2, -35, Math.toRadians(270)), Math.toRadians(270),
                    SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(120),
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(20)
            )
            .build();
    public static Trajectory rightPixelDrop = TrajectoryBuilder.buildTrajectory(
                    new Pose2d(-39.5, -62, Math.toRadians(270)))
            .back(1e-2)
            .splineToSplineHeading(new Pose2d(-39.5, -50, Math.toRadians(180)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(160)), Math.toRadians(90))
            .build();


    public static Trajectory driveToBoardLeft = TrajectoryBuilder.buildTrajectory(leftPixelDrop.end())
            .splineToSplineHeading(new Pose2d(-29.8, -56.0, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(leftBoardScore, Math.toRadians(0))
            .build();

    public static Trajectory driveToBoardCenter = TrajectoryBuilder.buildTrajectory(centerPixelDrop.end())
            .splineToSplineHeading(new Pose2d(-34, -56, Math.toRadians(180)), Math.toRadians(345))
            .splineToSplineHeading(new Pose2d(-15, -56, Math.toRadians(180)), Math.toRadians(0))
            .back(30)
            .splineToSplineHeading(new Pose2d(47.7, -34, Math.toRadians(180)), Math.toRadians(0))
            .build();


    public static Trajectory driveToBoardRight = TrajectoryBuilder.buildTrajectory(rightPixelDrop.end(), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(-35.6, -56), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-20, -57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(-2.3, -57, Math.toRadians(180)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(47.7, -41, Math.toRadians(180)), Math.toRadians(0))
            .build();






}
