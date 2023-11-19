package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.IntSupplier;

public class ElevatorToPosition extends CommandBase {
    private final ElevatorSubsystem elevator;
    ElevatorSubsystem.ElevatorPosition position;

    IntSupplier level;

    //use this for teleop
    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.ElevatorPosition position) {
        this.elevator = elevator;
        this.position = position;
        level = null;
    }

    public ElevatorToPosition(ElevatorSubsystem elevator, IntSupplier level){
        this.elevator = elevator;
        this.level = level;
        position = ElevatorSubsystem.ElevatorPosition.STOW_POSITION;

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setUsePID(true);
        //todo: find setpoint


        if(level != null){
            switch(level.getAsInt()){
                case 1:
                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_ONE;
                    break;
                case 2:
                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_TWO;
                    break;
                case 3:
                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_THREE;
                    break;
            }
        }

        elevator.setSetpoint(position.getValue());
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();

    }
}
