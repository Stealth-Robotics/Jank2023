package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ZeroHeadingWithDistanceSensors extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController rotationalController;

    public ZeroHeadingWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        rotationalController = new PIDController(0.025, 0, 0);
        rotationalController.setTolerance(3);
        rotationalController.setSetPoint(0);

        addRequirements(driveSubsystem, distanceSensorSubsystem);
    }

    @Override
    public void execute() {

        double rotationalError = distanceSensorSubsystem.getRightDistanceMillimeters() - distanceSensorSubsystem.getLeftDistanceMillimeters();
        double calculation = rotationalController.calculate(rotationalError);
        calculation = MathUtils.clamp(calculation, -0.3, 0.3);

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
