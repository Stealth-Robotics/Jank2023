package org.firstinspires.ftc.teamcode.commands.presets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPositionMotionProfiling;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem.ElevatorPosition;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class ScorePreset extends ParallelCommandGroup {

    public ScorePreset(ElevatorSubsystem elevator, ClawperSubsystem claw, IntSupplier level){
        addRequirements(elevator, claw);

        FtcDashboard.getInstance().getTelemetry().addData("level recieved", level.getAsInt());
        ElevatorPosition elevatorPosition = ElevatorPosition.LEVEL_ONE;
        switch (level.getAsInt()){
            case 1:
                elevatorPosition = ElevatorPosition.LEVEL_ONE;
                break;
            case 2:
                elevatorPosition = ElevatorPosition.LEVEL_TWO;
                break;
            case 3:
                elevatorPosition = ElevatorPosition.LEVEL_THREE;

                break;
            default:
                elevatorPosition = ElevatorPosition.LEVEL_ONE;

                break;
        }

        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ElevatorToPosition(elevator, elevatorPosition)
                ),
                new InstantCommand(() -> claw.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_SCORE))
            )
        );
    }

    public ScorePreset(ElevatorSubsystem elevator, ClawperSubsystem claw){
        this(elevator, claw, () -> 1);
        addRequirements(elevator, claw);

    }
}
