package org.firstinspires.ftc.teamcode.commands.presets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem.ElevatorPosition;
import org.stealthrobotics.library.commands.WaitBeforeCommand;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;

public class ScorePreset extends ParallelCommandGroup {

    public ScorePreset(ElevatorSubsystem elevator, ClawperSubsystem claw, IntSupplier level){
        addRequirements(elevator, claw);
        addCommands(
            new ParallelCommandGroup(
                    new ElevatorToPosition(elevator, level).withTimeout(2000),
                    new WaitBeforeCommand(500, new InstantCommand(() -> claw.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_SCORE))
                    )

            )
        );
    }
}
