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
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.AutoToTeleStorage;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Red far DELAY cycle auto", group="red auto", preselectTeleOp = "BLUE | Tele-Op")
public class RedFarDelayCycle extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;
    IntakeSubsystem intakeSubsystem;

    DistanceSensorSubsystem distance;



    @Override
    public void initialize() {
            Trajectory e = RedLeftTrajectories.leftPixelDrop;
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
        FtcDashboard.getInstance().getTelemetry().addData("heading measurment",
                Math.toDegrees(drive.getPoseEstimate().getHeading()));
        FtcDashboard.getInstance().getTelemetry().update();
        Trajectory pixelDrop = RedLeftTrajectories.centerPixelDrop;
        Trajectory board = RedLeftTrajectories.driveToBoardCenter;
        RedLeftTrajectories.Position position = RedLeftTrajectories.Position.CENTER;
        RedLeftTrajectories.Position whitePosition = RedLeftTrajectories.Position.RIGHT;
        FtcDashboard.getInstance().getTelemetry().addLine("fast load works");
        drive.setPoseEstimate(RedLeftTrajectories.leftPixelDrop.start());
        clawper.clawperClosedPosition();

        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        double yOffset = 0;
        switch (camera.getPosition()) {
            case "center":
                pixelDrop = RedLeftTrajectories.centerPixelDrop;
                board = RedLeftTrajectories.driveToBoardCenter;
                position = RedLeftTrajectories.Position.CENTER;
                whitePosition = RedLeftTrajectories.Position.RIGHT;
                yOffset = 4;
                break;
            case "right":
                pixelDrop = RedLeftTrajectories.rightPixelDrop;
                board = RedLeftTrajectories.driveToBoardRight;
                position = RedLeftTrajectories.Position.RIGHT;
                whitePosition = RedLeftTrajectories.Position.LEFT;

                break;
            case "left":
                pixelDrop = RedLeftTrajectories.leftPixelDrop;
                board = RedLeftTrajectories.driveToBoardLeft;

                position = RedLeftTrajectories.Position.LEFT;
                whitePosition = RedLeftTrajectories.Position.RIGHT;
                break;
        }

        return new SequentialCommandGroup(
                //all init stuff
                new InstantCommand(() -> elevator.setUsePID(true)),
                new InstantCommand(() -> elevator.setToCurrentPosition()),
                new InstantCommand(() -> elevator.setPower(0)),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new InstantCommand(() -> elevator.resetEncoderZero()),

                //TODO: FIND THIS
                new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level5Height + 0.005)),
                new FollowTrajectory(drive, pixelDrop),
//                new WaitCommand(500),

                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new FollowTrajectorySequence(drive, RedLeftTrajectories.firstStackIntake(position)),
                        new RunCommand(() -> intakeSubsystem.setSpeed(1))

                ),
//                new ParallelCommandGroup(
////                        new FollowTrajectorySequence(drive, RedLeftTrajectories.groundPickup),
//                        new WaitBeforeCommand(800, new InstantCommand(() -> intakeSubsystem.setHeight(0.2)))
//                ),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.setSpeed(0)),
                new WaitCommand(4000),

                new ParallelCommandGroup(
                        new FollowTrajectory(drive, board),
                        new WaitBeforeCommand(500, new InstantCommand(() -> intakeSubsystem.setSpeed(-1))),
                        new WaitBeforeCommand(1500, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 2)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.86,
                        AlignTranslationWithDistanceSensors.SensorSide.LEFT).withTimeout(1000)),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(350),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(400, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level4Height)),
                        new FollowTrajectory(drive, RedLeftTrajectories.driveToStack(position, 0)),
                        new StowPreset(elevator, clawper),
                        new WaitBeforeCommand(2000, new InstantCommand(() -> intakeSubsystem.setSpeed(1)))
                ),

                new WaitCommand(300),
                new ParallelCommandGroup(
                        new WaitBeforeCommand(1000, new InstantCommand(() -> intakeSubsystem.setSpeed(-1))),

                        new FollowTrajectory(drive, RedLeftTrajectories.dropTwoWhites(whitePosition, yOffset)),
                        new WaitBeforeCommand(3000, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 4)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.86,
                        AlignTranslationWithDistanceSensors.SensorSide.RIGHT).withTimeout(1000)),
                new InstantCommand(() -> AutoToTeleStorage.finalAutoHeading =
                        drive.getPoseEstimate().getHeading() + Math.PI / 2.0),
                new WaitCommand(50),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(125),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(300, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),
                new WaitCommand(0)





        );

    }
}
