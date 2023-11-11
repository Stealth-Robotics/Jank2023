package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends CommandBase {
    private DoubleSupplier leftY, leftX, rightX;
    private BooleanSupplier halfSpeed;
    private DriveSubsystem driveSubsystem;

    public DriveDefaultCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier halfSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.halfSpeed = halfSpeed;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveTeleop(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), halfSpeed.getAsBoolean());

    }
}
