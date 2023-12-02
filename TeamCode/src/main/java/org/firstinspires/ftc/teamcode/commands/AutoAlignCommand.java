package org.firstinspires.ftc.teamcode.commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends SequentialCommandGroup {

    private final DriveSubsystem drive;
    private final DistanceSensorSubsystem distance;
    private final DoubleSupplier leftY;




    public AutoAlignCommand(DriveSubsystem drive, DistanceSensorSubsystem distance, DoubleSupplier leftY) {
        this.drive = drive;
        this.distance = distance;
        this.leftY = leftY;

        addCommands(
                new ZeroHeadingWithDistanceSensors(drive, distance).withTimeout(2500),
                new AlignTranslationWithDistanceSensors(drive, distance),
                new ZeroHeadingWithDistanceSensors(drive, distance).withTimeout(1000),
                new DriveDefaultCommand(drive, leftY, () -> 0.0, () -> 0.0, () -> true)
        );

    }

}
