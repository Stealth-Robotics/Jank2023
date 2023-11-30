package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem.ElevatorPosition;

import java.util.function.BooleanSupplier;

public class StowPreset extends ParallelCommandGroup {

    public StowPreset(ElevatorSubsystem elevator, ClawperSubsystem claw, BooleanSupplier cancel) {
        addRequirements(elevator, claw);
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorToPosition(elevator, ElevatorPosition.STOW_POSITION).withTimeout(1000),
                                new RunCommand(() -> elevator.setPower(-1)).withTimeout(1000),
                                new InstantCommand(() -> elevator.setToCurrentPosition())


                        ),
                        new InstantCommand(() -> claw.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_STOW)),
                        new SequentialCommandGroup(
                            new InstantCommand(() -> claw.clawperRelease()),
                            new InstantCommand(() -> claw.clawperClosedPosition())),
                            new InstantCommand(() -> elevator.setPower(0)
                        )

                )
        );
    }

    public StowPreset(ElevatorSubsystem elevator, ClawperSubsystem claw) {
        this(elevator, claw, () -> false);
    }
}
