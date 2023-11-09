package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

    private final DistanceSensor rightDistance;
    private final DistanceSensor leftDistance;

    private final PIDController rightDrivePID = new PIDController(0, 0, 0);
    private final PIDController leftDrivePID = new PIDController(0, 0, 0);


    private boolean usePID = false;

    double headingOffset = 0;

    IMU imu;

    private SampleMecanumDrive roadrunnerDrive;

    public DriveSubsystem(HardwareMap hardwareMap, SampleMecanumDrive roadrunnerDrive) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");

        rightDistance = hardwareMap.get(DistanceSensor.class, "distanceRight");
        leftDistance = hardwareMap.get(DistanceSensor.class, "distanceLeft");

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
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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



    public double getRightDistanceMillimeters(){
        return rightDistance.getDistance(DistanceUnit.MM);
    }
    public double getLeftDistanceMillimeters(){

        return leftDistance.getDistance(DistanceUnit.MM);
    }

    public void setUseAutoAlignPID(boolean usePID){
        this.usePID = usePID;
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
        double speedMultiplier = halfSpeed ? 0.3 : 1.0;

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
        if(usePID){
            double rightPower = MathUtils.clamp(rightDrivePID.calculate(getRightDistanceMillimeters()), -0.2, 0.2);
            double leftPower = MathUtils.clamp(leftDrivePID.calculate(getLeftDistanceMillimeters()), -0.2, 0.2);


            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);
            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
        }
    }
}
