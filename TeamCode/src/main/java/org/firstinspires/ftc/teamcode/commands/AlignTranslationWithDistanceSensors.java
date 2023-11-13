package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AlignTranslationWithDistanceSensors extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController translationController;

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double setpoint)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(0.02, 0, 0);
        translationController.setTolerance(5);
        translationController.setSetPoint(setpoint);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    @Override
    public void execute() {
        double average = (distanceSensorSubsystem.getLeftDistanceMillimeters() + distanceSensorSubsystem.getRightDistanceMillimeters()) / 2;
        double calculation = translationController.calculate(average);
        calculation = MathUtils.clamp(calculation, -0.3, 0.3);

        driveSubsystem.driveTeleop(calculation, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return translationController.atSetPoint();
    }
}
