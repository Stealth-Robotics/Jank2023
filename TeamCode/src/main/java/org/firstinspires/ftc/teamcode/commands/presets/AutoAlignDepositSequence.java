package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.stealthrobotics.library.commands.WaitBeforeCommand;

public class AutoAlignDepositSequence extends SequentialCommandGroup {
    public AutoAlignDepositSequence(ElevatorSubsystem elevator, ClawperSubsystem claw, DriveSubsystem drive, DistanceSensorSubsystem distance){
        addRequirements(elevator, claw, drive, distance);

        addCommands(
                new AlignTranslationWithDistanceSensors(drive, distance, 1.84, AlignTranslationWithDistanceSensors.SensorSide.RIGHT).withTimeout(1000),
                new WaitCommand(250),
                new InstantCommand(() -> claw.clawperRelease()),
                new InstantCommand(() -> claw.clawperRelease()),

                new WaitBeforeCommand(300, new InstantCommand(() -> claw.rotatinToggle())),
                new InstantCommand(() -> claw.rotatinToggle())
        );
    }
}
