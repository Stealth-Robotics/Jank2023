package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveWhileAlignedCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftY;

    private final DistanceSensorSubsystem distance;

    private final PIDController headingController;

    public DriveWhileAlignedCommand(DriveSubsystem drive, DistanceSensorSubsystem distance, DoubleSupplier leftY) {
        this.drive = drive;
        this.leftY = leftY;
        this.distance = distance;

        headingController = new PIDController(-5, 0, 0);
        headingController.setTolerance(0.1);
        headingController.setSetPoint(0);

        addRequirements(drive, distance);
    }

    @Override
    public void execute() {
        double rotationalError = (distance.getAnalogRight() - distance.getAnalogLeft());
        double calculation = headingController.calculate(rotationalError);
        calculation = MathUtils.clamp(calculation, -0.3, 0.3);
        drive.driveTeleop(leftY.getAsDouble(), 0, calculation, true);
    }
}
