package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.presets.AutoAlignDepositSequence;
import org.firstinspires.ftc.teamcode.commands.presets.DriveToBoardSequence;
import org.firstinspires.ftc.teamcode.commands.presets.DriveToStackSequence;
import org.firstinspires.ftc.teamcode.commands.presets.ScorePreset;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Red far cycle auto", group="red auto")
public class RedFarCycleAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;
    IntakeSubsystem intakeSubsystem;

    DistanceSensorSubsystem distance;



    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap, () -> intakeSubsystem.getIntakeSpeed() != 0);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED, "left");
        distance = new DistanceSensorSubsystem(hardwareMap);
        elevator.setUsePID(false);
        register(drive, elevator, clawper);
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
        elevator.setUsePID(false);
    }

    @Override
    public Command getAutoCommand() {
        FtcDashboard.getInstance().getTelemetry().addLine("fast load works");
        drive.setPoseEstimate(RedLeftTrajectories.leftPixelDrop.start());
        clawper.clawperClosedPosition();
        return new SequentialCommandGroup(
                //all init stuff
                new InstantCommand(() -> elevator.setUsePID(true)),
                new InstantCommand(() -> elevator.setToCurrentPosition()),
                new InstantCommand(() -> elevator.setPower(0)),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new ElevatorReset(elevator),


                new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level5Height)),
                new FollowTrajectory(drive, RedLeftTrajectories.leftPixelDrop),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new FollowTrajectorySequence(drive, RedLeftTrajectories.leftFirstStackIntake),
                        new RunCommand(() -> intakeSubsystem.setSpeed(1))

                ),
                new ParallelCommandGroup(
                    new FollowTrajectorySequence(drive, RedLeftTrajectories.groundPickup),
                    new WaitBeforeCommand(500, new InstantCommand(() -> intakeSubsystem.setHeight(0.2)))
                ),

                new DriveToBoardSequence(drive, RedLeftTrajectories.yellowAndWhiteBoardDropLeft, intakeSubsystem, elevator, clawper, 2),

                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.84, AlignTranslationWithDistanceSensors.SensorSide.LEFT).withTimeout(1000)),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(500),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitBeforeCommand(400, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),

                new DriveToStackSequence(drive, intakeSubsystem, elevator, clawper, RedLeftTrajectories.Position.LEFT,
                        intakeSubsystem.level4Height, 0.5),


                new WaitCommand(1000),
                new DriveToBoardSequence(drive, RedLeftTrajectories.dropTwoWhites(RedLeftTrajectories.Position.RIGHT), intakeSubsystem, elevator, clawper, 2),

                new WaitCommand(100),
                new AutoAlignDepositSequence(elevator, clawper, drive, distance),

                new WaitCommand(500),
                new DriveToStackSequence(drive, intakeSubsystem, elevator, clawper, RedLeftTrajectories.Position.RIGHT,
                        0.2, 1.25),


                new WaitCommand(500),
                new DriveToBoardSequence(drive, RedLeftTrajectories.dropTwoWhites(RedLeftTrajectories.Position.RIGHT), intakeSubsystem, elevator, clawper, 3),

                new WaitCommand(100),
                new AutoAlignDepositSequence(elevator, clawper, drive, distance)



                );
    }
}
