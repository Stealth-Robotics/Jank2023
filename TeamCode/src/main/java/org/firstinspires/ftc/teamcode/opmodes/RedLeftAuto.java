//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.commands.FollowTrajectory;
//import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
//import org.firstinspires.ftc.teamcode.commands.RunIntakeForTime;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
//import org.stealthrobotics.library.commands.EndOpModeCommand;
//import org.stealthrobotics.library.commands.WaitBeforeCommand;
//import org.stealthrobotics.library.opmodes.StealthOpMode;
//@SuppressWarnings("unused")
//@Autonomous()
//public abstract class RedLeftAuto extends StealthOpMode {
//    DriveSubsystem drive;
//    SampleMecanumDrive mecanumDrive;
//    ElevatorSubsystem elevator;
//    ClawperSubsystem clawper;
//    IntakeSubsystem intake;
//
//
//    @Override
//    public void initialize() {
//        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
//        elevator = new ElevatorSubsystem(hardwareMap);
//        clawper = new ClawperSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);
//
//        register(drive, elevator, clawper, intake);
//    }
//
//    @Override
//    public void whileWaitingToStart() {
//        CommandScheduler.getInstance().run();
//    }
//
//    @Override
//    public Command getAutoCommand() {
//        drive.setPoseEstimate(RedLeftTrajectories.leftPixelDrop.start());
//        //Set trajectory sequence based on camera, implement later
//
//        Trajectory pixelDrop = RedLeftTrajectories.leftPixelDrop;
//
//        return new SequentialCommandGroup(
//                new FollowTrajectory(drive, pixelDrop),
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
//
//
//
//                new EndOpModeCommand(this)
//        );
//    }
//
//
//}
