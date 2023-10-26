package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Trajectory trajectory;

    public FollowTrajectory(DriveSubsystem driveSubsystem, Trajectory trajectory){
        this.driveSubsystem = driveSubsystem;
        this.trajectory = trajectory;
        addRequirements(driveSubsystem);
    }
    //async trajectory following to use other things while following
    @Override
    public void initialize(){
        driveSubsystem.followTrajectoryAsync(trajectory);
    }
    @Override
    public void execute(){
        driveSubsystem.update();
    }
    @Override
    public boolean isFinished(){
        return driveSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.stop();
    }

}
