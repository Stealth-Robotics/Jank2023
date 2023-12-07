package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;

import java.util.function.DoubleSupplier;

public class ClawperDefault extends CommandBase {
    private final ClawperSubsystem clawper;
    private final DoubleSupplier leftY;

    public ClawperDefault(ClawperSubsystem clawper, DoubleSupplier leftY){
        this.clawper = clawper;
        this.leftY = leftY;
        addRequirements(clawper);
    }

    @Override
    public void execute() {
        clawper.rotationToPosition(clawper.getRotation() + leftY.getAsDouble() * 0.01);
    }
}
