package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends CommandBase {
    private DoubleSupplier leftY, leftX, rightX;
    private BooleanSupplier halfSpeed, autoAlign;
    private DriveSubsystem driveSubsystem;

    public DriveDefaultCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier halfSpeed, BooleanSupplier autoAlign) {
        this.driveSubsystem = driveSubsystem;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.halfSpeed = halfSpeed;
        this.autoAlign = autoAlign;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        //allows strafe movement if joystick is pushed in that direction
        //sets pid to false to not interfere with manual control
        if(autoAlign.getAsBoolean() && Math.abs(leftY.getAsDouble()) > 0.1) {
            driveSubsystem.setUseAutoAlignPID(false);
            driveSubsystem.driveTeleop(leftY.getAsDouble(), 0, 0, false);
        }
        //otherwise, if button is pressed and no joystick input, auto align
        else if(autoAlign.getAsBoolean()){
            driveSubsystem.setUseAutoAlignPID(true);

        }
        //otherwise, drive normally
        else {
            driveSubsystem.setUseAutoAlignPID(false);

            driveSubsystem.driveTeleop(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), halfSpeed.getAsBoolean());
        }
    }
}
