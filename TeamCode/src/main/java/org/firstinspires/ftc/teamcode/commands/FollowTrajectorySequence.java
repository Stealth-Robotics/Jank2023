package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class FollowTrajectorySequence extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private TrajectorySequence trajectorySequence;

    public FollowTrajectorySequence(DriveSubsystem driveSubsystem, TrajectorySequence trajectorySequence){
        this.driveSubsystem = driveSubsystem;
        this.trajectorySequence = trajectorySequence;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        driveSubsystem.followTrajectorySequenceAsync(trajectorySequence);
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
