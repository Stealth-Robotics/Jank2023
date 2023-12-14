package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionProcessorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Config

public class ZeroHeadingWithDistanceSensors extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController rotationalController;

    public static double kP = 1.35 , kI = 1.45, kD = 0.05;

    public ZeroHeadingWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        rotationalController = new PIDController(kP, kI, kD);

        rotationalController.setTolerance(0.005);
        rotationalController.setSetPoint(0);

        addRequirements(driveSubsystem, distanceSensorSubsystem);
    }

    @Override
    public void initialize() {
        rotationalController.reset();
    }

    @Override
    public void execute() {

        double rotationalError = (distanceSensorSubsystem.getAnalogRight() - distanceSensorSubsystem.getAnalogLeft());
        double calculation = rotationalController.calculate(rotationalError);


        FtcDashboard.getInstance().getTelemetry().addData("Rotational Error", rotationalError);
        FtcDashboard.getInstance().getTelemetry().addData("Controller Error", rotationalController.getPositionError());
        FtcDashboard.getInstance().getTelemetry().addData("Calculation", calculation);
        FtcDashboard.getInstance().getTelemetry().update();

        driveSubsystem.driveTeleop(0,0, calculation,false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return rotationalController.atSetPoint();
    }
}
