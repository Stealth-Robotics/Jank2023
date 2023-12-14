package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
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
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@Autonomous(name="Red close auto", group="red auto")
public class RedRightAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;
    IntakeSubsystem intakeSubsystem;

    DistanceSensorSubsystem distance;

    String trustedSensor;

    double distanceStrafe = 0;


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
        Trajectory pixelDrop = RedCloseTrajectories.rightPixelDrop;

        Trajectory board = RedCloseTrajectories.driveToBoardRight;
        Trajectory park = TrajectoryBuilder.buildTrajectory(RedCloseTrajectories.driveToBoardRight.end())
                .strafeLeft(10)
                .build();


        drive.setPoseEstimate(RedCloseTrajectories.rightPixelDrop.start());



        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        switch (camera.getPosition()) {
            case "center":
                pixelDrop = RedCloseTrajectories.centerPixelDrop;
                board = RedCloseTrajectories.driveToBoardCenter;
                distanceStrafe = 25;
                trustedSensor = "left";
                break;
            case "right":
                pixelDrop = RedCloseTrajectories.rightPixelDrop;
                board = RedCloseTrajectories.driveToBoardRight;
                distanceStrafe = 18;
                trustedSensor = "left";

                break;
            case "left":
                pixelDrop = RedCloseTrajectories.leftPixelDrop;
                board = RedCloseTrajectories.driveToBoardLeft;
                distanceStrafe = 30;
                trustedSensor = "right";
                break;
        }
        camera.stopCamera();

                return new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.setUsePID(true)),
                        new InstantCommand(() -> elevator.setToCurrentPosition()),
                        new InstantCommand(() -> elevator.setPower(0)),
                        new InstantCommand(() -> clawper.rotatinToggle()),
                        new ElevatorReset(elevator),

                        new ParallelCommandGroup(
                                new FollowTrajectory(drive, pixelDrop)

                        ),
            new WaitCommand(500),
                        new InstantCommand(() -> clawper.clawperRelease()),
                        new WaitCommand(250),
                        new ParallelCommandGroup(
                            new FollowTrajectory(drive, board),
                                new WaitBeforeCommand(500, new ScorePreset(elevator, clawper, () -> 1))
                        ),



                        new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.87).withTimeout(4000)),

                        new WaitCommand(500),
                        new InstantCommand(() -> clawper.clawperRelease()),
                        new WaitBeforeCommand(500, new InstantCommand(() -> clawper.rotatinToggle())),
                        new InstantCommand(() -> clawper.rotatinToggle()),
                        new WaitCommand(250),
                        new FollowTrajectory(drive,
                                TrajectoryBuilder.buildTrajectory(board.end())
                                        .strafeLeft(distanceStrafe)

                                        .build()
                        ),

                        new StowPreset(elevator, clawper),
                        new ElevatorReset(elevator),
                        new EndOpModeCommand(this)




                );
        }



}
