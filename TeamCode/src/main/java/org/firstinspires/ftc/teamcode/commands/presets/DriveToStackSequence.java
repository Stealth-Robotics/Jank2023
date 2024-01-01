package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.stealthrobotics.library.commands.WaitBeforeCommand;

public class DriveToStackSequence extends ParallelCommandGroup {

    public DriveToStackSequence(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
                                ElevatorSubsystem elevatorSubsystem, ClawperSubsystem clawperSubsystem,
                                RedLeftTrajectories.Position sideFrom, double intakeHeight, double yOffset){

        addRequirements(driveSubsystem, intakeSubsystem, elevatorSubsystem, clawperSubsystem);
        addCommands(
                new InstantCommand(() -> intakeSubsystem.setHeight(intakeHeight)),
                new FollowTrajectory(driveSubsystem, RedLeftTrajectories.driveToStack(sideFrom, yOffset)),
                new StowPreset(elevatorSubsystem, clawperSubsystem),
                new WaitBeforeCommand(2000, new InstantCommand(() -> intakeSubsystem.setSpeed(1)))

        );
    }




}
