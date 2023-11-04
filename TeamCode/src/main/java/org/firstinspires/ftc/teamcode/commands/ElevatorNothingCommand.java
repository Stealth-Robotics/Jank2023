package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorNothingCommand extends CommandBase {

    ElevatorSubsystem elevator;

    public ElevatorNothingCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
