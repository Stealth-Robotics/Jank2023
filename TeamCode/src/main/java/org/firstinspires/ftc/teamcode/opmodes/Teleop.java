package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        CameraSubsystem camera = new CameraSubsystem(hardwareMap, Alliance.RED);



//        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
//        driveSubsystem = new DriveSubsystem(hardwareMap, roadrunnerDrive);




//        driverGamepad = new GamepadEx(gamepad1);
//        operatorGamepad = new GamepadEx(gamepad2);
//
//        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator,
//                () -> (operatorGamepad.gamepad.right_trigger - operatorGamepad.gamepad.left_trigger)
//        ));
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
