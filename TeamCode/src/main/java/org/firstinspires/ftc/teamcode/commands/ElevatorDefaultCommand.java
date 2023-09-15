package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends CommandBase {
    private DoubleSupplier input;
    private ElevatorSubsystem elevator;

    //Teleop Default Command
    public ElevatorDefaultCommand(ElevatorSubsystem elevator, DoubleSupplier input) {
        this.elevator = elevator;
        this.input = input;
        addRequirements(elevator);
    }

    //Autonomous Default Command
    public ElevatorDefaultCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if(input != null && input.getAsDouble() > 0.05) {
            elevator.setPower(input.getAsDouble());
            elevator.setUsePID(false);
            //sets elevator to hold in place after manual control, pid won't run until trigger is released
            elevator.setToCurrentPosition();
        }
        else{
            elevator.setUsePID(true);
        }
    }
}
