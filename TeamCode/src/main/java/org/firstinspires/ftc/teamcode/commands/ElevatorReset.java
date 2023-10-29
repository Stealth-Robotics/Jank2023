package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

public class ElevatorReset extends CommandBase {

    private final ElevatorSubsystem elevator;
    private final BooleanSupplier endItAll;

    public ElevatorReset(ElevatorSubsystem elevator, BooleanSupplier cancel){
        this.elevator = elevator;
        endItAll = cancel;
        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setSlowly();
        FtcDashboard.getInstance().getTelemetry().addData("running ", true);
        FtcDashboard.getInstance().getTelemetry().update();

    }



    @Override
    //ends command if elevator is at zero velocity or if the cancel button is pressed
    public boolean isFinished() {
        return elevator.checkZeroVelocity() || endItAll.getAsBoolean();
    }

    @Override
    //runs code to reset encoder position
    public void end(boolean interrupted) {
        elevator.resetElevatorStall();
    }
}
