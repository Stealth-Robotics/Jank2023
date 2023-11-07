package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.RunIntakeForTime;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.TrajectorySequenceBuilder;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;
@SuppressWarnings("unused")
@Autonomous(name="Red left auto", group="red auto")
public class RedLeftAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    CameraSubsystem camera;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
//    IntakeSubsystem intake;


    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        elevator = new ElevatorSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);



        register(drive, camera, clawper, elevator);

        clawper.clawperClosedPosition();

    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        drive.setPoseEstimate(RedLeftTrajectories.leftPixelDrop.start());
        //Set trajectory sequence based on camera, implement later
        Trajectory pixelDrop = RedLeftTrajectories.leftPixelDrop;

        Trajectory board = RedLeftTrajectories.driveToBoardLeft;




        telemetry.addData("pos: ", camera.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("pos", camera.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
        switch(camera.getPosition()){
            case "center":
                pixelDrop = RedLeftTrajectories.centerPixelDrop;
                board = RedLeftTrajectories.driveToBoardCenter;
                break;
            case "right":
                pixelDrop = RedLeftTrajectories.rightPixelDrop;
                board = RedLeftTrajectories.driveToBoardRight;
                break;
            case "left":
                pixelDrop = RedLeftTrajectories.leftPixelDrop;
                board = RedLeftTrajectories.driveToBoardLeft;


        }


        return new SequentialCommandGroup(
            new InstantCommand(() -> clawper.rotatinToggle()),
            new ParallelCommandGroup(
                new FollowTrajectory(drive, pixelDrop),
                    new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.AUTO_SCORE)
            ),
            new InstantCommand(() -> clawper.rotatinToggle()),
            new WaitCommand(500),
            new InstantCommand(() -> clawper.clawperRelease()),
            new WaitCommand(1000),
            new StowPreset(elevator, clawper),
            new FollowTrajectory(drive, board)
            //new WaitCommand(1000),
            //new FollowTrajectory(drive, board)

//                new ParallelCommandGroup(
//                        //waits 1 second before driving backwards while outtaking hexes
//                        new SequentialCommandGroup(
//                                new WaitBeforeCommand(
//                                        1000,
//                                        new FollowTrajectory(drive, RedLeftTrajectories.outtakeDriveBack)
//                                )
//                        ),
//                        new RunIntakeForTime(intake, 2000, true)
//                ),



        );
    }


}
