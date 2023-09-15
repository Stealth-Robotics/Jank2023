package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorScorePreset extends CommandBase {
    private ElevatorSubsystem elevator;
    private DoubleSupplier input;

    //use this for teleop
    public ElevatorScorePreset(ElevatorSubsystem elevator, DoubleSupplier input) {
        this.elevator = elevator;
        this.input = input;
    }

    //use this for autonomous
    public ElevatorScorePreset(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setUsePID(true);
        //todo: find setpoint
        elevator.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return (elevator.atSetpoint() || (input != null && input.getAsDouble() > 0.05));

    }
}
