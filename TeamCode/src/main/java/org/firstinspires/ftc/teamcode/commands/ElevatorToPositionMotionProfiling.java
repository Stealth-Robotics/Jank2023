package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

public class ElevatorToPositionMotionProfiling extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final double position;
    private final BooleanSupplier cancel;
    private boolean cancelled = false;
    //command to use in teleop, pass in button to cancel command
    public ElevatorToPositionMotionProfiling(ElevatorSubsystem elevator, double position, BooleanSupplier cancel){
        this.elevator = elevator;
        this.position = position;
        this.cancel = cancel;
        addRequirements(elevator);
    }
    //use in auto
    public ElevatorToPositionMotionProfiling(ElevatorSubsystem elevator, double position){
        this(elevator, position, null);
    }


    @Override
    public void initialize(){
        //sets goal position to whatever user specifies, with 0 velo and accel
        elevator.setUseMotionProfiling(true);
        elevator.setUsePID(true);
        elevator.runToPositionMotionProfiling(new MotionState(position, 0, 0),
                System.currentTimeMillis() / (long)1000);
    }

    @Override
    public boolean isFinished(){
        //checks if button is pressed if not null, if pressed sets cancelled to true if wanting to
        //use any different end conditions if cancelled
        //otherwise, just returns if elevator has reached its position
        if(cancel != null && cancel.getAsBoolean()){
            cancelled = true;
            return true;
        }
        return elevator.motionProfilingAtSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        //sets motion profiling to false to use position hold pid
        //sets elevator to current position
        elevator.setUseMotionProfiling(false);
        elevator.setToCurrentPosition();
    }

}
