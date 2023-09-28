package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorScorePreset;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    DriveSubsystem driveSubsystem;

    SampleMecanumDrive roadrunnerDrive;
    ElevatorSubsystem elevatorSubsystem;


    IntakeSubsystem intakeSubsystem;

    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;


    @Override
    public void initialize() {
        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        driveSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveSubsystem,
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getRightX(),
                        () -> driverGamepad.gamepad.right_bumper
                )
        );

        elevatorSubsystem.setDefaultCommand(
                new ElevatorDefaultCommand(elevatorSubsystem, () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger))
        );

        intakeSubsystem.setDefaultCommand(
                new IntakeDefaultCommand(intakeSubsystem, () -> (driverGamepad.gamepad.right_trigger - driverGamepad.gamepad.left_trigger))
        );

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ElevatorScorePreset(elevatorSubsystem,
                        () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)));
    }
}
