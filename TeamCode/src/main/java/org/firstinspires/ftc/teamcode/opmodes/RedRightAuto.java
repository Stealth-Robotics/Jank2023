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
import org.firstinspires.ftc.teamcode.commands.AutoAlignCommand;
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
import org.firstinspires.ftc.teamcode.trajectories.RedLeftTrajectories;
import org.firstinspires.ftc.teamcode.trajectories.RedRightTrajectories;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@Autonomous(name="Red right auto", group="red auto")
public class RedRightAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;

    DistanceSensorSubsystem distance;


    @Override
    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, Alliance.RED);
        distance = new DistanceSensorSubsystem(hardwareMap);
        register(drive, elevator, clawper);
    }

    @Override
    public void whileWaitingToStart() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public Command getAutoCommand() {
        Trajectory pixelDrop = RedRightTrajectories.rightPixelDrop;

        Trajectory board = RedRightTrajectories.driveToBoardRight;

        drive.setPoseEstimate(RedRightTrajectories.rightPixelDrop.start());


        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        switch (camera.getPosition()) {
//            case "center":
//                pixelDrop = RedLeftTrajectories.centerPixelDrop;
//                board = RedLeftTrajectories.driveToBoardCenter;
//                break;
//            case "right":
//                pixelDrop = RedRightTrajectories.rightPixelDrop;
//                board = RedRightTrajectories.driveToBoardRight;
//                break;
//            case "left":
//                pixelDrop = RedLeftTrajectories.leftPixelDrop;
        }

                return new SequentialCommandGroup(
                        new InstantCommand(() -> clawper.rotatinToggle()),
                        new ParallelCommandGroup(
                                new FollowTrajectory(drive, pixelDrop)
//                    new ElevatorToPosition(elevator, ElevatorSubsystem.ElevatorPosition.AUTO_SCORE)
                        ),
//            new InstantCommand(() -> clawper.rotatinToggle()),
//            new WaitCommand(500),
                        new InstantCommand(() -> clawper.clawperRelease()),
                        new WaitCommand(1000),

                        new FollowTrajectory(drive, board),



                        new WaitBeforeCommand(500, new AlignTranslationWithDistanceSensors(drive, distance, 60).withTimeout(4000)),
                        new ScorePreset(elevator, clawper, () -> 2),
                        new WaitCommand(500),
                        new InstantCommand(() -> clawper.clawperRelease())


                );
        }



}
