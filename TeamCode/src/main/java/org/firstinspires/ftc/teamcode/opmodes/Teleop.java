package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;



public abstract class Teleop extends StealthOpMode {
    //DriveSubsystem driveSubsystem;

    //SampleMecanumDrive roadrunnerDrive;

    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;

    DriveSubsystem driveSubsystem;
    SampleMecanumDrive roadrunnerDrive;

    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;

    private CameraSubsystem cameraSubsystem;


    public void initialize() {
        cameraSubsystem = new CameraSubsystem(hardwareMap, Alliance.RED);

        telemetry.addData("pos:", cameraSubsystem.getPosition());



        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap);




        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        driveSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveSubsystem,
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getRightX(),
                        () -> driverGamepad.gamepad.right_bumper,
                        () -> driverGamepad.gamepad.left_bumper
                )
        );

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> driveSubsystem.resetAngle()));
        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator,
                () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)
        ));
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> elevator.resetEncoderZero())
//        );
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> driveSubsystem.resetAngle()));
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whileHeld(
//                new DriveToBoard(driveSubsystem)
//        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_STOW))
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_SCORE))
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> clawper.clawperToPosition(ClawperSubsystem.ClawperPosition.RELEASE_BOTH))
        );

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                    new InstantCommand(() -> clawper.clawperClosedPosition())
            );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> clawper.clawperRelease())
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_CLIMB)));

    }
    @SuppressWarnings("unused")
    //sets the camera to the red prop processor if alliance is red
    @TeleOp(name = "RED | Tele-Op", group = "Red")
    public static class RedTeleop extends Teleop {


    }
    //sets the camera to the blue prop processor if alliance is blue
    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Tele-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {

    }
}
