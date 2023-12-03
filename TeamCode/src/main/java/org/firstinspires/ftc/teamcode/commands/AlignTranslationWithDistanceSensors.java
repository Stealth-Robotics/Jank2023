package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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

    public static double kP = 0.5, kI = 0.0, kD = 0.0;
    public static double setpoint = 1.77;


    double averageRight = 0;
    double averageLeft = 0;


    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.03);
        translationController.setSetPoint(setpoint);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double position)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.02);
        translationController.setSetPoint(position);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    @Override
    public void execute() {
        translationController.setSetPoint(1.8 - distanceSensorSubsystem.getDistanceOffset());
        double average = (distanceSensorSubsystem.getAnalogRight() + distanceSensorSubsystem.getAnalogLeft()) / 2;
        double calculation = translationController.calculate(average);

        calculation = MathUtils.clamp(-calculation, -0.2, 0.2);

        driveSubsystem.setMotors(calculation);

        telemetry.addData("sp", translationController.getSetPoint());
        telemetry.update();
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
