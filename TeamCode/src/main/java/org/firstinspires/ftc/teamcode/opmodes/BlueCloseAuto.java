package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AlignTranslationWithDistanceSensors;
import org.firstinspires.ftc.teamcode.commands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
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
import org.firstinspires.ftc.teamcode.trajectories.BlueCloseTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.TrajectoryBuilder;
import org.firstinspires.inspection.InspectionState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.SaveAutoHeadingCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@Autonomous(name="blue close", preselectTeleOp = "BLUE | Tele-Op")
public class BlueCloseAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;
    IntakeSubsystem intakeSubsystem;

    DistanceSensorSubsystem distance;

    double distanceStrafe = 0;


    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap, () -> intakeSubsystem.getIntakeSpeed() != 0);
        camera = new CameraSubsystem(hardwareMap, Alliance.BLUE, "right");
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
        Trajectory pixelDrop = BlueCloseTrajectories.rightDrop;

        Trajectory board = BlueCloseTrajectories.boardRight;
        Trajectory park = TrajectoryBuilder.buildTrajectory(BlueCloseTrajectories.boardRight.end())
                .strafeRight(10)
                .build();


        drive.setPoseEstimate(BlueCloseTrajectories.rightDrop.start());



        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        switch (camera.getPosition()) {
            case "center":
                pixelDrop = BlueCloseTrajectories.centerDrop;
                board = BlueCloseTrajectories.boardCenter;
                distanceStrafe = 20;

                break;
            case "right":
                pixelDrop = BlueCloseTrajectories.rightDrop;
                board = BlueCloseTrajectories.boardRight;
                distanceStrafe = 18;

                break;
            case "left":
                pixelDrop = BlueCloseTrajectories.leftDrop;
                board = BlueCloseTrajectories.driveToBoardLeft;
                distanceStrafe = 30;

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
//                    new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.AUTO_SCORE)
                ),
//            new InstantCommand(() -> clawper.rotatinToggle()),
//            new WaitCommand(500),
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
                                .splineToSplineHeading(new Pose2d(55, 12, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                ),

                new StowPreset(elevator, clawper),
                new ElevatorReset(elevator),

                new SaveAutoHeadingCommand(() -> drive.getPoseEstimate().getHeading()),
                new EndOpModeCommand(this)





        );
    }



}
