package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftY;

    private final PIDController rotationController = new PIDController(0.01, 0, 0);
    private final PIDController translationController = new PIDController(0.01, 0, 0);

    public AutoAlignCommand(DriveSubsystem drive, DoubleSupplier leftY) {
        this.drive = drive;
        this.leftY = leftY;

        rotationController.setSetPoint(0);
        translationController.setSetPoint(200);

        rotationController.setTolerance(5);
        translationController.setTolerance(5);
    }

    public AutoAlignCommand(DriveSubsystem drive) {
        this(drive, null);
    }

    @Override
    public void execute() {
        if(leftY != null && Math.abs(leftY.getAsDouble()) > 0.1) {
            drive.driveTeleop(leftY.getAsDouble(), 0, 0, false);
        }
        else {
            drive.driveTeleop(
                    0,
                    translationController.calculate(
                            -(drive.getRightDistanceMillimeters() + drive.getLeftDistanceMillimeters()
                            )/2),
                    rotationController.calculate(drive.getRightDistanceMillimeters() - drive.getLeftDistanceMillimeters()),
                    false
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
