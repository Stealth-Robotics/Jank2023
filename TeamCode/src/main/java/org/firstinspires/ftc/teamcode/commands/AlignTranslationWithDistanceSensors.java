package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Config

public class AlignTranslationWithDistanceSensors extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController translationController;

    public static double kP = 0.05, kI = 0.0, kD = 0.0;


    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double setpoint)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(15);
        translationController.setSetPoint(setpoint);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    @Override
    public void execute() {
        double average = (distanceSensorSubsystem.getLeftDistanceMillimeters() + distanceSensorSubsystem.getRightDistanceMillimeters()) / 2;
        double calculation = translationController.calculate(average);
        calculation = MathUtils.clamp(calculation, -0.2, 0.2);

        driveSubsystem.setMotors(calculation);
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
