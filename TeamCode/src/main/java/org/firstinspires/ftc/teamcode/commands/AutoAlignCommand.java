package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftY;

    private final PIDController leftPID = new PIDController(0, 0, 0);
    private final PIDController rightPID = new PIDController(0, 0, 0);

    public AutoAlignCommand(DriveSubsystem drive, DoubleSupplier leftY) {
        this.drive = drive;
        this.leftY = leftY;

        //TODO: FIND CORRECT DISTANCE VALUES
        leftPID.setSetPoint(0);
        rightPID.setSetPoint(0);

        leftPID.setTolerance(1);
        rightPID.setTolerance(1);
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
            drive.setLeftPower(MathUtils.clamp(leftPID.calculate(drive.getLeftDistanceMillimeters()), -0.2, 0.2));
            drive.setRightPower(MathUtils.clamp(rightPID.calculate(drive.getRightDistanceMillimeters()), -0.2, 0.2));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
