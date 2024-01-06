package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {
    DoubleSupplier triggerInput;

    DoubleSupplier heightAdjust = null;
    IntakeSubsystem intake;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier triggerInput) {
        this.triggerInput = triggerInput;
        this.intake = intake;
        addRequirements(intake);
    }

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier triggerInput, DoubleSupplier heightAdjust) {
        this.triggerInput = triggerInput;
        this.intake = intake;
        this.heightAdjust = heightAdjust;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(heightAdjust != null){
            double calculation = intake.getHeight() - heightAdjust.getAsDouble() * 0.01;
            if(calculation < 0.2){
                calculation = 0.2;
            }
            intake.setHeight(calculation);
        }
        if (Math.abs(triggerInput.getAsDouble()) > 0.1) intake.setSpeed(triggerInput.getAsDouble());
        else{
            intake.setSpeed(0);
        }
    }
}
