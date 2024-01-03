package org.firstinspires.ftc.teamcode.commands.presets;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.stealthrobotics.library.commands.WaitBeforeCommand;

public class DriveToBoardSequence extends ParallelCommandGroup {

    public DriveToBoardSequence(DriveSubsystem drive, Trajectory trajectory, IntakeSubsystem intake){
//        addRequirements(drive, intake);

        addCommands(
                new FollowTrajectory(drive, trajectory),
                new WaitBeforeCommand(2000, new InstantCommand(() -> intake.setSpeed(0)))

        );
    }
}
