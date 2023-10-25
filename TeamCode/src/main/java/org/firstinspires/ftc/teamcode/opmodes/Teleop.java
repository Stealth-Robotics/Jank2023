package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToBoard;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.BluePropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.RedPropProcessor;
import org.stealthrobotics.library.opmodes.StealthOpMode;



public abstract class Teleop extends StealthOpMode {
    DriveSubsystem driveSubsystem;

    SampleMecanumDrive roadrunnerDrive;
    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;

    private CameraSubsystem cameraSubsystem;
    public CameraSubsystem intializeCameraSubsystem(){
        return null;
    }


    public void initialize() {

        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
        cameraSubsystem = intializeCameraSubsystem();



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

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> driveSubsystem.resetAngle()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                new DriveToBoard(driveSubsystem)
        );

    }
    @SuppressWarnings("unused")
    //sets the camera to the red prop processor if alliance is red
    @TeleOp(name = "RED | Tele-Op", group = "Red")
    public static class RedTeleop extends Teleop {
        @Override
        public CameraSubsystem intializeCameraSubsystem() {
            return new CameraSubsystem(hardwareMap, new RedPropProcessor());
        }


    }
    //sets the camera to the blue prop processor if alliance is blue
    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Tele-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {
        @Override
        public CameraSubsystem intializeCameraSubsystem() {
            return new CameraSubsystem(hardwareMap, new BluePropProcessor());
        }
    }
}
