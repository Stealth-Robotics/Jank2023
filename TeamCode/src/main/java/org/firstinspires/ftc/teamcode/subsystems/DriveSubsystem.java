package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    double headingOffset = 0;

    BNO055IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //TODO: CHECK DIRECTIONS
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public double getAngle(){
        return -imu.getAngularOrientation().firstAngle + headingOffset;
    }

    public void resetAngle(){
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

    public void driveTeleop(double leftStickY, double leftStickX, double rightStickX, boolean halfSpeed){
        // This code is pulled from Game Manual 0
        // https://gm0.org/en/latest/docs/software/mecanum-drive.html
        double speedMultiplier = halfSpeed ? 0.5 : 1.0;

        double y = -leftStickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing
        double rotation = rightStickX;
        double rotx = x;
        double roty = y;
        double botHeading = getAngle();
        //gets heading from imu every loop, reversed as imu heading is cw positive

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
