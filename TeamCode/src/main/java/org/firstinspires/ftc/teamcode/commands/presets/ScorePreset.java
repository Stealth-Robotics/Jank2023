package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ElevatorToPositionMotionProfiling;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem.ElevatorPosition;

import java.util.function.BooleanSupplier;

public class ScorePreset extends ParallelCommandGroup {

    public ScorePreset(ElevatorSubsystem elevator, ClawperSubsystem claw, BooleanSupplier cancel){
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ElevatorToPositionMotionProfiling(elevator, ElevatorPosition.SCORE_POSITION)
                ),
                new InstantCommand(() -> claw.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_SCORE))
            )
        );
    }

    public ScorePreset(ElevatorSubsystem elevator, ClawperSubsystem claw){
        this(elevator, claw, () -> false);
    }
}
