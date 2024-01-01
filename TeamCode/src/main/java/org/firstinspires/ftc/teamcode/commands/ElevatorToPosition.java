package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.IntSupplier;


@Config
public class ElevatorToPosition extends CommandBase {
    private final ElevatorSubsystem elevator;
    ElevatorSubsystem.ElevatorPosition position;

    public static int positionIncrement = 120;

    double setpoint = 0;

    IntSupplier level;

    //use this for teleop
    public ElevatorToPosition(ElevatorSubsystem elevator, ElevatorSubsystem.ElevatorPosition position) {
        this.elevator = elevator;
        this.position = position;
        level = null;
    }

    public ElevatorToPosition(ElevatorSubsystem elevator, IntSupplier level) {
        this.elevator = elevator;
        this.level = level;
        position = ElevatorSubsystem.ElevatorPosition.STOW_POSITION;

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setUsePID(true);
        //todo: find setpoint


        if (level != null) {
            setpoint = ElevatorSubsystem.ElevatorPosition.LEVEL_ONE.getValue() + (positionIncrement * (level.getAsInt() - 1));
//            switch(level.getAsInt()){
//                case 1:
//                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_ONE;
//                    break;
//                case 2:
//                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_TWO;
//                    break;
//                case 3:
//                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_THREE;
//                    break;
//                case 4:
//                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_FOUR;
//                    break;
//                case 5:
//                    position = ElevatorSubsystem.ElevatorPosition.LEVEL_FIVE;
//                    break;
            elevator.setSetpoint(setpoint);

        }
        else{
            elevator.setSetpoint(position.getValue());
        }


    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();

    }
}
