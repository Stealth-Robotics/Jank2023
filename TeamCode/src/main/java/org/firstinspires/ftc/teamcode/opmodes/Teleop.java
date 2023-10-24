package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    //DriveSubsystem driveSubsystem;

    //SampleMecanumDrive roadrunnerDrive;

    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;

    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;


    @Override
    public void initialize() {
//        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
//        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
//
//
        elevator = new ElevatorSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap);


        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator,
                () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)
        ));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> elevator.resetEncoderZero())
        );

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> clawper.clawperToPosition(ClawperSubsystem.ClawperPosition.RELEASE_BOTH))
        );

        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> clawper.clawperRelease())
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> clawper.clawperClosedPosition())
        );

//        driveSubsystem.setDefaultCommand(
//                new DriveDefaultCommand(
//                        driveSubsystem,
//                        () -> driverGamepad.getLeftY(),
//                        () -> driverGamepad.getLeftX(),
//                        () -> driverGamepad.getRightX(),
//                        () -> driverGamepad.gamepad.right_bumper,
//                        () -> driverGamepad.gamepad.left_bumper
//                )
//        );
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> driveSubsystem.resetAngle()));
    }
    @SuppressWarnings("unused")
    @TeleOp(name = "RED | Tele-Op", group = "Red")
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Tele-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }
}
