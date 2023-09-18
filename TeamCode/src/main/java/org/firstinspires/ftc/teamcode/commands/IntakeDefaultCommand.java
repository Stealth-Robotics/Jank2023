package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {
    DoubleSupplier triggerInput;
    IntakeSubsystem intake;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier triggerInput) {
        this.triggerInput = triggerInput;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (Math.abs(triggerInput.getAsDouble()) > 0.05) intake.setSpeed(triggerInput.getAsDouble());
    }
}
