package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveToBoard extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    public DriveToBoard(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveTowardBoardSlow();
    }

    @Override
    public boolean isFinished() {
        //will stop when distance is between 3 and 5 millimeters
        return (driveSubsystem.getDistance() > 3 && driveSubsystem.getDistance() < 5);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
