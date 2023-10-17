package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    //PID Constants
    //TODO: Tune PID
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kF = 0;
    private boolean useMotionProfiling = false;
    private final PIDFController elevatorPID = new PIDFController(kP, kI, kD, kF);
    private final com.acmerobotics.roadrunner.control.PIDFController motionProfilePID =
            new com.acmerobotics.roadrunner.control.PIDFController(new PIDCoefficients(0, 0, 0));
    private final double SPEED_LIMIT  = 0.25;

    //MOTION PROFILING CONSTANTS
    //VALUES ARE IN TICKS/SECOND (EQUIVALENT FOR ACCEL AND JERK)
    private final double MAX_VELOCITY = 0.0;
    private final double MAX_ACCEL = 0.0;
    private final double MAX_JERK = 0.0;
    //TODO: TUNE TOLERANCE
    private final double MOTION_PROFILE_TOLERANCE = 10.0;

    private long motionProfileStartTime = 0;
    private long elapsedTime = 0;

    private MotionProfile profile;
    private boolean usePID = true;

    private double currentVelo = 0;
    private double lastVelo = 0;
    private boolean firstLoop = true;
    private double deltaV = 0;

    private double accel = 0;

    private double deltaT = 0;

    private long currentTime = 0;
    private long lastTime = 0;
    public ElevatorSubsystem(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "elevatorMotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "elevatorMotor2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorPID.setTolerance(10);

    }

    public void setSetpoint(double pos) {
        elevatorPID.setSetPoint(pos);
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetPoint();
    }

    public double getEncoderPosition() {
        return motor1.getCurrentPosition();
    }

    public void setToCurrentPosition() {
        elevatorPID.setSetPoint(getEncoderPosition());
    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public double getVelo(){
        return motor1.getVelocity();
    }

    public double getAccel(){
        return  accel;
    }

    public boolean motionProfilingAtSetpoint(){
        return (Math.abs(motionProfilePID.getLastError()) < MOTION_PROFILE_TOLERANCE);
    }

    public void setUseMotionProfiling(boolean useMotionProfiling){
        this.useMotionProfiling = useMotionProfiling;
    }
    //runs to position using motion profiling
    //generates motion profile based on current state and user specified goalState
    //also must take in start time for motion profile to profile
    public void runToPositionMotionProfiling(MotionState goalState, long timeStarted){
        motionProfileStartTime = timeStarted / (long)1000;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getEncoderPosition(), getVelo(), getAccel()),
                goalState,
                MAX_VELOCITY,
                MAX_ACCEL
        );
    }

    //TODO: Tune PID
    //call setUsePID(true) to use PID
    @Override
    public void periodic() {
        currentVelo = getVelo();
        currentTime = System.currentTimeMillis() / (long)1000;
        if(firstLoop){
            lastTime = currentTime;
            lastVelo = currentVelo;
            firstLoop = false;
        }
        deltaT = currentTime - lastTime;
        lastTime = currentTime;
        deltaV = currentVelo - lastVelo;
        lastVelo = currentVelo;

        accel = deltaV / deltaT;

        if (usePID) {
            if(useMotionProfiling){
                elapsedTime = System.currentTimeMillis() - motionProfileStartTime;
                MotionState state = profile.get(elapsedTime);
                motionProfilePID.setTargetVelocity(state.getV());
                motionProfilePID.setTargetAcceleration(state.getA());
                motionProfilePID.setTargetPosition(state.getX());
                double power = motionProfilePID.update(getEncoderPosition());
                setPower(MathUtils.clamp(power, -SPEED_LIMIT, SPEED_LIMIT));
            }
            else {
                double power = elevatorPID.calculate(getEncoderPosition());
                setPower(power);
            }
        }

    }
}
