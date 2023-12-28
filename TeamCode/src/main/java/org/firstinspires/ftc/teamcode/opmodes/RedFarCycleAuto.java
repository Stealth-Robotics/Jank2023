package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.presets.ScorePreset;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedCloseTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.EndOpModeCommand;
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
        drive.setPoseEstimate(RedLeftTrajectories.leftPixelDrop.start());
        return new SequentialCommandGroup(
                //all init stuff
                new InstantCommand(() -> elevator.setUsePID(true)),
                new InstantCommand(() -> elevator.setToCurrentPosition()),
                new InstantCommand(() -> elevator.setPower(0)),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new ElevatorReset(elevator),

                //TODO: FIND THIS
                new InstantCommand(() -> intakeSubsystem.setHeight(-1)),
                new FollowTrajectory(drive, RedLeftTrajectories.leftPixelDrop),
                new WaitCommand(500),

                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(250),
                new ParallelDeadlineGroup(
                        new FollowTrajectory(drive, RedLeftTrajectories.leftFirstStackIntake),
                        new RunCommand(() -> intakeSubsystem.setSpeed(1))

                ).andThen(new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                new WaitCommand(100),
                new ParallelCommandGroup(
                    new FollowTrajectory(drive, RedLeftTrajectories.yellowAndWhiteBoardDropLeft),
                    new WaitBeforeCommand(3000, new ScorePreset(elevator, clawper, () -> 2))
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.87).withTimeout(4000)),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(300, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),

                new FollowTrajectory(drive, RedLeftTrajectories.driveToStack(RedLeftTrajectories.Position.LEFT)),
                new WaitCommand(500),
                new FollowTrajectory(drive, RedLeftTrajectories.dropTwoWhites(RedLeftTrajectories.Position.RIGHT)),
                new WaitCommand(500),

                new FollowTrajectory(drive, RedLeftTrajectories.driveToStack(RedLeftTrajectories.Position.RIGHT)),
                new WaitCommand(500),

                new FollowTrajectory(drive, RedLeftTrajectories.dropTwoWhites(RedLeftTrajectories.Position.RIGHT))


                );
    }
}
