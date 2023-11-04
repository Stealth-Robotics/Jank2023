package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorToPosition extends CommandBase {
    private final ElevatorSubsystem elevator;
    ElevatorSubsystem.ElevatorPosition position;

    //use this for teleop
    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.ElevatorPosition position) {
        this.elevator = elevator;
        this.position = position;
    }


    @Override
    public void initialize() {
        elevator.setUsePID(true);
        //todo: find setpoint
        elevator.setSetpoint(position.getValue());
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();

    }
}
