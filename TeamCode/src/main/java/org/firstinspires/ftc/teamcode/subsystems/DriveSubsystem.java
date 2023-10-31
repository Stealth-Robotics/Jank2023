package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class DriveSubsystem extends SubsystemBase {
    FtcDashboard dashboard  = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;

    //private final DistanceSensor distanceSensor;
    double headingOffset = 0;

    IMU imu;

    private SampleMecanumDrive roadrunnerDrive;

    public DriveSubsystem(HardwareMap hardwareMap, SampleMecanumDrive roadrunnerDrive) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        //TODO: CHECK DIRECTIONS
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );


        imu.initialize(parameters);

        this.roadrunnerDrive = roadrunnerDrive;

    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset + Math.PI;
    }

    public void resetAngle() {
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void followTrajectoryAsync(Trajectory trajectory){
        roadrunnerDrive.followTrajectoryAsync(trajectory);
    }

    public void update(){
        roadrunnerDrive.update();
    }

    public boolean isBusy(){
        return roadrunnerDrive.isBusy();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectory){
        roadrunnerDrive.followTrajectorySequenceAsync(trajectory);
    }

    public void setPoseEstimate(Pose2d poseEstimate){
        roadrunnerDrive.setPoseEstimate(poseEstimate);
    }

    public void driveTowardBoardSlow(){
        driveTeleop(0.2, 0, 0, false, false);
    }

//    public double getDistance(){
//        return distanceSensor.getDistance(DistanceUnit.MM);
//    }

    public Pose2d getPoseEstimate(){
        return roadrunnerDrive.getPoseEstimate();
    }

    public void stop(){
        driveTeleop(0,0,0, false, false);
    }

    public void driveTeleop(double leftStickY, double leftStickX, double rightStickX, boolean halfSpeed, boolean strafe) {
        // This code is pulled from Game Manual 0
        // https://gm0.org/en/latest/docs/software/mecanum-drive.html
        double speedMultiplier = halfSpeed ? 0.5 : 1.0;

        Vector2d inputVector = new Vector2d(
                -leftStickY,
                -leftStickX
        ).rotated(getAngle());
        roadrunnerDrive.setWeightedDrivePower(
                new Pose2d(
                        inputVector.getX() * speedMultiplier,
                        -inputVector.getY() * speedMultiplier,
                        -rightStickX * speedMultiplier
                )
        );
        roadrunnerDrive.update();
    }

    @Override
    public void periodic() {
    }
}
