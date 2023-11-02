package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.RunIntakeForTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
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
//    ElevatorSubsystem elevator;
//    ClawperSubsystem clawper;
//    IntakeSubsystem intake;


    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
//        elevator = new ElevatorSubsystem(hardwareMap);
//        clawper = new ClawperSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);

        register(drive, camera);
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
        Trajectory driveBack = RedLeftTrajectories.outtakeDriveBackLeft;
        Trajectory board = RedLeftTrajectories.driveToBoardLeft;

        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        switch(camera.getPosition()){
            case "center":
                pixelDrop = RedLeftTrajectories.centerPixelDrop;
                driveBack = RedLeftTrajectories.outtakeDriveBackCenter;
                board = RedLeftTrajectories.driveToBoardCenter;
                break;
            case "right":
                pixelDrop = RedLeftTrajectories.rightPixelDrop;
                driveBack = RedLeftTrajectories.outtakeDriveBackRight;
                board = RedLeftTrajectories.driveToBoardRight;
                break;
            case "left":
                pixelDrop = RedLeftTrajectories.leftPixelDrop;
                driveBack = RedLeftTrajectories.outtakeDriveBackLeft;
                board = RedLeftTrajectories.driveToBoardLeft;


        }


        return new SequentialCommandGroup(
            new FollowTrajectory(drive, pixelDrop),
            //new WaitCommand(1000),
            new FollowTrajectory(drive, driveBack),
            //new WaitCommand(1000),
            new FollowTrajectory(drive, board)

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
