package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

public class ElevatorReset extends CommandBase {

    private final ElevatorSubsystem elevator;


    public ElevatorReset(ElevatorSubsystem elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setUsePID(false);
        elevator.setSlowly();
        FtcDashboard.getInstance().getTelemetry().addData("running ", true);
        FtcDashboard.getInstance().getTelemetry().update();

    }



    @Override
    //ends command if elevator is at zero velocity or if the cancel button is pressed
    public boolean isFinished() {
        return elevator.checkZeroVelocity();
    }

    @Override
    //runs code to reset encoder position
    public void end(boolean interrupted) {

        elevator.resetElevatorStall();
        elevator.setUsePID(true);
        elevator.setSetpoint(elevator.getEncoderPosition());
    }
}
