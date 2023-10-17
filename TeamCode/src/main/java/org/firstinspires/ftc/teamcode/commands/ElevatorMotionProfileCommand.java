package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.Timer;

public class ElevatorMotionProfileCommand extends CommandBase {
    private final MotionProfile profile;
    private final ElevatorSubsystem elevator;

    private final double position;

    PIDFController controller;
    private long startTime;
    private long elapsedTime;

    //TODO: TUNE POSITION TOLERANCE
    private final double TOLERANCE = 10;

    public ElevatorMotionProfileCommand(ElevatorSubsystem elevator, double position){
        this.elevator = elevator;
        this.position = position;
        //TODO: TUNE
        controller = new PIDFController(new PIDCoefficients(0, 0, 0));
        //TODO: FIND MAX VEL, ACCEL, AND JERK VALUES
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(elevator.getEncoderPosition(), elevator.getVelo(), elevator.getAccel()),
                new MotionState(position, 0, 0),
                0,
                0,
                0
        );

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setUsePID(false);
        startTime = System.nanoTime();

    }

    @Override
    public void execute() {
        elapsedTime = System.nanoTime() - startTime;
        elapsedTime *= (long) Math.pow(10, -9);
        MotionState state = profile.get(elapsedTime);
        controller.setTargetPosition(state.getX());
        controller.setTargetAcceleration(state.getA());
        controller.setTargetVelocity(state.getV());
        double power = controller.update(elevator.getEncoderPosition());
        elevator.setPower(power);
    }
    @Override
    public boolean isFinished() {
        return (elevator.getEncoderPosition() == position + TOLERANCE ||
                elevator.getEncoderPosition() == position - TOLERANCE);
    }
    @Override
    public void end(boolean isFinished){
        elevator.setUsePID(true);
        elevator.setToCurrentPosition();
    }
}
