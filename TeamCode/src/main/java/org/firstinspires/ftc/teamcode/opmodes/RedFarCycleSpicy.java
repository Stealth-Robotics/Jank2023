package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.commands.presets.ScorePreset;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedCloseTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.trajectories.TrajectorySequenceBuilder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Red far cycle thru middle", group="red auto")
public class RedFarCycleSpicy extends StealthOpMode {
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

                //TODO: FIND THIS
                new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level5Height)),
                new FollowTrajectory(drive, RedLeftTrajectories.leftPixelDrop),
//                new WaitCommand(500),

                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new FollowTrajectorySequence(drive, RedLeftTrajectories.leftFirstStackIntake),
                        new RunCommand(() -> intakeSubsystem.setSpeed(1))

                ),


                new ParallelCommandGroup(
                        new FollowTrajectory(drive, RedLeftTrajectories.yellowAndWhiteBoardDropLeftSpicyPath),
                        new WaitBeforeCommand(3000, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 2)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.84,
                        AlignTranslationWithDistanceSensors.SensorSide.LEFT).withTimeout(1000)),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(200),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(400, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level4Height)),
                        new FollowTrajectory(drive, RedLeftTrajectories.driveToStackSpicyPath(RedLeftTrajectories.Position.LEFT, 0.5)),
                        new StowPreset(elevator, clawper),
                        new WaitBeforeCommand(2000, new InstantCommand(() -> intakeSubsystem.setSpeed(1)))
                ),

                new WaitCommand(300),
                new ParallelCommandGroup(
                        new FollowTrajectory(drive, RedLeftTrajectories.dropTwoWhitesSpicyPath(RedLeftTrajectories.Position.RIGHT)),
                        new WaitBeforeCommand(3000, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 3)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.84,
                        AlignTranslationWithDistanceSensors.SensorSide.RIGHT).withTimeout(1000)),
                new WaitCommand(0),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(75),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(300, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new WaitCommand(0),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeSubsystem.setHeight(0.2)),
                        new FollowTrajectory(drive, RedLeftTrajectories.driveToStackSpicyPath(RedLeftTrajectories.Position.RIGHT, 1.25)),
                        new StowPreset(elevator, clawper),
                        new WaitBeforeCommand(2000, new InstantCommand(() -> intakeSubsystem.setSpeed(1)))
                ),

                new WaitCommand(300),
                new ParallelCommandGroup(
                        new FollowTrajectory(drive, RedLeftTrajectories.dropTwoWhitesSpicyPath(RedLeftTrajectories.Position.RIGHT)),
                        new WaitBeforeCommand(3000, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 3)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.84,
                        AlignTranslationWithDistanceSensors.SensorSide.RIGHT).withTimeout(1000)),
                new WaitCommand(0),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(75),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(300, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new StowPreset(elevator, clawper)



        );
    }
}
