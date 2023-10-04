package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class DriveSubsystem extends SubsystemBase {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    double headingOffset = 0;

    IMU imu;

    private SampleMecanumDrive roadrunnerDrive;

    public DriveSubsystem(HardwareMap hardwareMap, SampleMecanumDrive roadrunnerDrive) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");

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
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );


        imu.initialize(parameters);

        this.roadrunnerDrive = roadrunnerDrive;

    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
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

    public Pose2d getPoseEstimate(){
        return roadrunnerDrive.getPoseEstimate();
    }

    public void stop(){
        driveTeleop(0,0,0, false);
    }

    public void driveTeleop(double leftStickY, double leftStickX, double rightStickX, boolean halfSpeed) {
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
                        inputVector.getY() * speedMultiplier,
                        -rightStickX * speedMultiplier
                )
        );

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        telemetry.addData("Heading", getAngle());
    }
}
