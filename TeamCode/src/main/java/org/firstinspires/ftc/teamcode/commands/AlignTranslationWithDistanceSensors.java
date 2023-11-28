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
    public static double setpoint = 1.25;


    double averageRight = 0;
    double averageLeft = 0;


    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.1);
        translationController.setSetPoint(setpoint);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    @Override
    public void execute() {
        averageLeft = 0;
        averageRight = 0;
        for(int i = 0; i < 5; i++){
            averageRight += distanceSensorSubsystem.getRightDistanceMillimeters();
            averageLeft += distanceSensorSubsystem.getLeftDistanceMillimeters();
        }
        averageLeft /= 5;
        averageRight /= 5;
        double average = (averageLeft + averageRight)/2;
        double calculation = translationController.calculate(distanceSensorSubsystem.getAnalog());
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
