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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectories.BlueFarTrajectories;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.AutoToTeleStorage;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Blue far SUPER DELAY cycle auto", group="red auto", preselectTeleOp = "BLUE | Tele-Op")
public class BlueSuperDelayAuto extends StealthOpMode {
    DriveSubsystem drive;
    SampleMecanumDrive mecanumDrive;
    ElevatorSubsystem elevator;
    ClawperSubsystem clawper;
    CameraSubsystem camera;
    IntakeSubsystem intakeSubsystem;

    DistanceSensorSubsystem distance;



    @Override
    public void initialize() {
            Trajectory e = BlueFarTrajectories.leftPixelDrop;
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, mecanumDrive);
        elevator = new ElevatorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        clawper = new ClawperSubsystem(hardwareMap, () -> intakeSubsystem.getIntakeSpeed() != 0);
        camera = new CameraSubsystem(hardwareMap, Alliance.BLUE, "left");
        camera.setRects(
            new Rect(
                    new Point(0, 0),
                    new Point(180, 480)
            ),


            new Rect(
                    new Point(180, 0),
                    new Point(450, 480)
            ),
            new Rect(
                    new Point(450, 0),
                    new Point(640, 480)
            )
        );
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
        Trajectory pixelDrop = BlueFarTrajectories.centerPixelDrop;
        Trajectory board = BlueFarTrajectories.driveToBoardCenter;
        TrajectorySequence stackIntake = BlueFarTrajectories.intakeBlueLeft;
        BlueFarTrajectories.Position position = BlueFarTrajectories.Position.CENTER;
        BlueFarTrajectories.Position whitePosition = BlueFarTrajectories.Position.RIGHT;
        FtcDashboard.getInstance().getTelemetry().addLine("fast load works");
        drive.setPoseEstimate(BlueFarTrajectories.leftPixelDrop.start());
        clawper.clawperClosedPosition();

        telemetry.addData("pos: ", camera.getPosition());
        telemetry.update();
        double yOffset = 0;
        switch (camera.getPosition()) {
            case "center":
                pixelDrop = BlueFarTrajectories.centerPixelDrop;
                board = BlueFarTrajectories.driveToBoardCenter;
                position = BlueFarTrajectories.Position.CENTER;
                whitePosition = BlueFarTrajectories.Position.LEFT;
                yOffset = 4;
                stackIntake = BlueFarTrajectories.firstStackIntake(position);
                break;
            case "right":
                pixelDrop = BlueFarTrajectories.rightPixelDrop;
                board = BlueFarTrajectories.driveToBoardRight;
                position = BlueFarTrajectories.Position.RIGHT;
                whitePosition = BlueFarTrajectories.Position.LEFT;
                stackIntake = BlueFarTrajectories.intakeBlueLeft;

                break;
            case "left":
                pixelDrop = BlueFarTrajectories.leftPixelDrop;
                board = BlueFarTrajectories.driveToBoardLeft;

                position = BlueFarTrajectories.Position.LEFT;
                whitePosition = BlueFarTrajectories.Position.RIGHT;

                stackIntake = BlueFarTrajectories.firstStackIntake(position);
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
                new InstantCommand(() -> intakeSubsystem.setHeight(intakeSubsystem.level5Height)),
                new FollowTrajectory(drive, pixelDrop),
//                new WaitCommand(500),

                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new FollowTrajectorySequence(drive, stackIntake),
                        new RunCommand(() -> intakeSubsystem.setSpeed(1))

                ),
                new WaitCommand(1000),
                new InstantCommand(() -> intakeSubsystem.setSpeed(0)),
                new WaitCommand(16000),
//                new ParallelCommandGroup(
////                        new FollowTrajectorySequence(drive, BlueFarTrajectories.groundPickup),
//                        new WaitBeforeCommand(800, new InstantCommand(() -> intakeSubsystem.setHeight(0.2)))
//                ),


                new ParallelCommandGroup(
                        new WaitBeforeCommand(500, new InstantCommand(() -> intakeSubsystem.setSpeed(-1))),
                        new WaitBeforeCommand(1500, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new FollowTrajectory(drive, board),
                        new WaitBeforeCommand(3000, new InstantCommand(() -> intakeSubsystem.setSpeed(0))),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 10),
                                new ScorePreset(elevator, clawper, () -> 2)
                        )
                ),
                new WaitBeforeCommand(100, new AlignTranslationWithDistanceSensors(drive, distance, 1.85,
                        AlignTranslationWithDistanceSensors.SensorSide.LEFT).withTimeout(1000)),
                new WaitCommand(250),
                new InstantCommand(() -> clawper.clawperRelease()),
                new WaitCommand(350),
                new InstantCommand(() -> clawper.clawperRelease()),

                new WaitBeforeCommand(400, new InstantCommand(() -> clawper.rotatinToggle())),
                new InstantCommand(() -> clawper.rotatinToggle()),

                new InstantCommand(() -> AutoToTeleStorage.finalAutoHeading =
                        drive.getPoseEstimate().getHeading() + Math.PI / 2.0)



        );

    }
}
