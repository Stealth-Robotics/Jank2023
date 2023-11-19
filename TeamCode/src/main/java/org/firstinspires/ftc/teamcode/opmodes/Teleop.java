package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorNothingCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ZeroHeadingWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.presets.ScorePreset;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;



public abstract class Teleop extends StealthOpMode {
    //DriveSubsystem driveSubsystem;

    //SampleMecanumDrive roadrunnerDrive;

    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;

    IntakeSubsystem intake;

    DriveSubsystem driveSubsystem;
    SampleMecanumDrive roadrunnerDrive;

    PlaneSubsystem plane;

    DistanceSensorSubsystem distance;

    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;

    //private CameraSubsystem cameraSubsystem;


    public void initialize() {
        //cameraSubsystem = new CameraSubsystem(hardwareMap, Alliance.RED);

        //telemetry.addData("pos:", cameraSubsystem.getPosition());
        //new InstantCommand(() -> elevator.resetEncoderZero());

        //new ElevatorReset(elevator);




        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        plane = new PlaneSubsystem(hardwareMap);
        distance = new DistanceSensorSubsystem(hardwareMap);

        clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_STOW);

//        telemetry.addData("level: ", elevator.getLevel());
//        telemetry.addData("elevator pos: ", elevator.getEncoderPosition());
//        telemetry.update();


        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        //TODO: ONLTY FOR TESTING, REMOVE LATER
        new InstantCommand(() -> elevator.setUsePID(false));
        driveSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveSubsystem,
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getRightX(),
                        () -> driverGamepad.gamepad.right_bumper
                )
        );

        intake.setDefaultCommand(
               new IntakeDefaultCommand(intake, () -> (driverGamepad.gamepad.right_trigger - driverGamepad.gamepad.left_trigger))
        );
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> clawper.clawperRelease()));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> driveSubsystem.resetAngle()));
        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator,
                () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)
        ));
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new ElevatorReset(elevator)
//        );
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new InstantCommand(() -> elevator.setSetpoint(500))
//        );

        //operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new StowPreset(elevator, clawper, () -> operatorGamepad.gamepad.b));
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> plane.setPlane(0.6)));
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whileHeld(
//                new DriveToBoard(driveSubsystem)
//        );
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new InstantCommand(() -> clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_STOW))
//        );
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new InstantCommand(() -> clawper.clawperToPosition(ClawperSubsystem.ClawperPosition.RELEASE_BOTH))
//        );
//
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                    new InstantCommand(() -> clawper.clawperClosedPosition())
//            );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> clawper.rotatinToggle())
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> elevator.incrementLevel(-1)));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> elevator.incrementLevel(1)));
        //operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ScorePreset(elevator, clawper, () -> elevator.getLevel()));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ElevatorReset(elevator));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> elevator.setSetpoint(500)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new SequentialCommandGroup(
                new ZeroHeadingWithDistanceSensors(driveSubsystem, distance),

//                new WaitCommand(500),
                new AlignTranslationWithDistanceSensors(driveSubsystem, distance, 45),

                new ZeroHeadingWithDistanceSensors(driveSubsystem, distance),
                new DriveDefaultCommand(driveSubsystem, () -> driverGamepad.getLeftY(), () -> 0, () -> 0, () -> true))
        );



        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> clawper.clawperClosedPosition()));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ScorePreset(elevator, clawper, () -> elevator.getLevel()));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new StowPreset(elevator, clawper));

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
