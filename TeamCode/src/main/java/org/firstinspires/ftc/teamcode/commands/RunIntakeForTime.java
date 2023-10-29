package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RunIntakeForTime extends SequentialCommandGroup {

    public RunIntakeForTime(IntakeSubsystem intake, long time, boolean reverse){
        addCommands(
                //todo: check if outtake speed is too fast
                new RunCommand(() -> intake.setSpeed(reverse ? -1 : 1), intake).withTimeout(time),
                new InstantCommand(() -> intake.setSpeed(0), intake)
        );
    }
}
