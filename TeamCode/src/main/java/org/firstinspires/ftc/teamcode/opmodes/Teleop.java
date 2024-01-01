package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.ClawperDefault;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ZeroHeadingWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.presets.ScorePreset;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionProcessorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;
import org.stealthrobotics.library.AutoToTeleStorage;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.HashMap;


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

    AprilTagVisionProcessorSubsystem camera;


    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
        elevator.setUsePID(false);
    }


    public void initialize() {
        //cameraSubsystem = new CameraSubsystem(hardwareMap, Alliance.RED);

        //telemetry.addData("pos:", cameraSubsystem.getPosition());
        //new InstantCommand(() -> elevator.resetEncoderZero());

        //new ElevatorReset(elevator);

        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap, () -> Math.abs(intake.getIntakeSpeed()) > 0.1);
        plane = new PlaneSubsystem(hardwareMap);
        distance = new DistanceSensorSubsystem(hardwareMap);
        camera = new AprilTagVisionProcessorSubsystem(hardwareMap);

        clawper.rotationToPosition(ClawperSubsystem.ClawperPosition.ROTATION_STOW);

        driveSubsystem.headingAfterAuto(AutoToTeleStorage.finalAutoHeading);

//        telemetry.addData("level: ", elevator.getLevel());
//        telemetry.addData("elevator pos: ", elevator.getEncoderPosition());
//        telemetry.update();


        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        //default commands

        driveSubsystem.setDefaultCommand(
                new DriveDefaultCommand(
                        driveSubsystem,
                        () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX(),
                        () -> driverGamepad.getRightX(),
                        () -> driverGamepad.gamepad.right_bumper
                )
        );

        clawper.setDefaultCommand(new ClawperDefault(clawper, () -> operatorGamepad.getLeftY()));

        intake.setDefaultCommand(
               new IntakeDefaultCommand(intake, () -> (driverGamepad.gamepad.right_trigger - driverGamepad.gamepad.left_trigger))
        );

        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator,
                () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)
        ));


        //drive commands
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(driveSubsystem::resetAngle));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new StowPreset(elevator, clawper).andThen(new ElevatorReset(elevator))

        );
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenHeld(
                new ZeroHeadingWithDistanceSensors(driveSubsystem, distance)
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()  -> distance.resetRightOffset()));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> distance.incrementRightOffset(.001)));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> distance.incrementRightOffset(-.001)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> distance.resetDistanceOffset()));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> distance.incrementDistance(.02)));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> distance.incrementDistance(-0.02)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> intake.testHeight()));



        //clawper commands
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(clawper::clawperRelease));
        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(clawper::clawperClosedPosition));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(clawper::rotatinToggle)
        );

        //elevator commands
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ScorePreset(elevator, clawper, () -> elevator.getLevel()));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new StowPreset(elevator, clawper).andThen(new ElevatorReset(elevator)));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ElevatorReset(elevator));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> elevator.incrementLevel(-1)));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> elevator.incrementLevel(1)));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ScorePreset(elevator, clawper, () -> elevator.getLevel())
        );
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new AutoAlignCommand(driveSubsystem, distance, () -> driverGamepad.getLeftY()),
                                new AlignTranslationWithDistanceSensors(driveSubsystem, distance),
                                () -> (distance.getAnalogRight() < 1.2)
                        ),
                    new ScorePreset(elevator, clawper, () -> elevator.getLevel())
                )
        );
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new SelectCommand(
//                        new HashMap<Object, Command>(){{
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_ONE, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_ONE));
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_TWO, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_TWO));
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_THREE, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_THREE));
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_FOUR, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_FOUR));
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_FIVE, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_FIVE));
//                            put(ElevatorSubsystem.ElevatorPosition.LEVEL_SIX, new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.LEVEL_SIX));
//                        }},
//
//                        elevator::getLevel
//                )
//        );




        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> plane.setPlane(0.6)));












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
