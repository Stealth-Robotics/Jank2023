package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorToPosition;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.presets.StowPreset;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.BlueLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@Autonomous(name="Blueleft")
public class BlueRightAuto extends StealthOpMode {
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
        camera = new CameraSubsystem(hardwareMap, Alliance.BLUE);
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
        drive.setPoseEstimate(BlueLeftTrajectories.leftPixelDrop.start());
        //Set trajectory sequence based on camera, implement later
        TrajectorySequence pixelDrop = BlueLeftTrajectories.leftPixelDrop;
        Trajectory driveBack = BlueLeftTrajectories.outtakeDriveBackLeft;
        Trajectory board = BlueLeftTrajectories.driveToBoardLeft;




        telemetry.addData("pos: ", camera.getPosition());
//        FtcDashboard.getInstance().getTelemetry().addData("pos", camera.getPosition());
//        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
        switch(camera.getPosition()){
            case "center":
                pixelDrop = BlueLeftTrajectories.centerPixelDrop;
                driveBack = BlueLeftTrajectories.outtakeDriveBackCenter;
                board = BlueLeftTrajectories.driveToBoardCenter;
                break;
            case "right":
                pixelDrop = BlueLeftTrajectories.rightPixelDrop;
                driveBack = BlueLeftTrajectories.outtakeDriveBackRight;
                board = BlueLeftTrajectories.driveToBoardRight;
                break;
            case "left":
                pixelDrop = BlueLeftTrajectories.leftPixelDrop;
                driveBack = BlueLeftTrajectories.outtakeDriveBackLeft;
                board = BlueLeftTrajectories.driveToBoardLeft;


        }


        return new SequentialCommandGroup(
            new InstantCommand(() -> clawper.rotatinToggle()),
            new ParallelCommandGroup(
                new FollowTrajectorySequence(drive, pixelDrop),
                    new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.AUTO_SCORE)
            ),
            new InstantCommand(() -> clawper.rotatinToggle()),
            new WaitCommand(500),
            new InstantCommand(() -> clawper.clawperRelease()),
            new WaitCommand(1000),
            new StowPreset(elevator, clawper),
            new FollowTrajectory(drive, driveBack)
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
