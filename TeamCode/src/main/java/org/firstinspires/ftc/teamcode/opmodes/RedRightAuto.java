package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.RedRightTrajectories;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@Autonomous(name="Red right auto", group="red auto")
public class RedRightAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
//    ElevatorSubsystem elevator;
//    ClawperSubsystem clawper;
//    IntakeSubsystem intake;


    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
//        elevator = new ElevatorSubsystem(hardwareMap);
//        clawper = new ClawperSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);

        register(drive);
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        drive.setPoseEstimate(RedRightTrajectories.rightPixelDrop.start());
        //Set trajectory sequence based on camera, implement later

        Trajectory pixelDrop = RedRightTrajectories.leftPixelDrop;

        return new SequentialCommandGroup(
            new FollowTrajectory(drive, RedRightTrajectories.rightPixelDrop),
            //new WaitCommand(1000),
            new FollowTrajectory(drive, RedRightTrajectories.outtakeDriveBackRight),
            //new WaitCommand(1000),
            new FollowTrajectorySequence(drive, RedRightTrajectories.driveToBoardRight)

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
