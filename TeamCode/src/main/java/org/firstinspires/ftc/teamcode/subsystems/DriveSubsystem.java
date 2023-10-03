package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetAngle() {
        imu.resetYaw();
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

    public void stop(){
        driveTeleop(0,0,0, false);
    }

    public void driveTeleop(double leftStickY, double leftStickX, double rightStickX, boolean halfSpeed) {
        // This code is pulled from Game Manual 0
        // https://gm0.org/en/latest/docs/software/mecanum-drive.html
        double speedMultiplier = halfSpeed ? 0.5 : 1.0;

        double y = -leftStickY; // Remember, this is reversed!
        double x = leftStickX;
        double rotation = rightStickX;

        double botHeading = getAngle();
        //gets heading from imu every loop, reversed as imu heading is cw positive



        double rotx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double roty = y * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotx = rotx * 1.1;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        //sets power of motors based on field-centric rotated values
        double denominator = Math.max(Math.abs(roty) + Math.abs(rotx) + Math.abs(rotation), 1);
        double leftFrontDrivePower = (roty + rotx + rotation) / denominator;
        double leftRearDrivePower = (roty - rotx + rotation) / denominator;
        double rightFrontDrivePower = (roty - rotx - rotation) / denominator;
        double rightRearDrivePower = (roty + rotx - rotation) / denominator;

        frontLeftMotor.setPower(leftFrontDrivePower * speedMultiplier);
        backLeftMotor.setPower(leftRearDrivePower * speedMultiplier);
        frontRightMotor.setPower(rightFrontDrivePower * speedMultiplier);
        backRightMotor.setPower(rightRearDrivePower * speedMultiplier);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        telemetry.addData("Heading", getAngle());
    }
}
