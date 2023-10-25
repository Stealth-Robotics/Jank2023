package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.math.filter.Debouncer;

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

    private final Debouncer stallDebouncer;

    public enum ElevatorPosition{
        SCORE_POSITION(0.0);


        private final double value;
        private ElevatorPosition(double value){
            this.value = value;
        }
        public double getValue(){
            return value;
        }
    }
    public ElevatorSubsystem(HardwareMap hardwareMap) {
        stallDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);
        motor1 = hardwareMap.get(DcMotorEx.class, "elevatorMotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "elevatorMotor2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorPID.setTolerance(10);

    }

    public void setSetpoint(double pos) {
        elevatorPID.setSetPoint(pos);
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetPoint();
    }

    public void resetStall(){

    }

    public double getEncoderPosition() {
        return motor1.getCurrentPosition();
    }
    public void resetEncoderZero(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setToCurrentPosition() {
        elevatorPID.setSetPoint(getEncoderPosition());
    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void setSlowly(){
        setPower(-0.1);
        stallDebouncer.calculate(false);
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public double getVelo(){
        return motor1.getVelocity();
    }

    public boolean motionProfilingAtSetpoint(){
        return (Math.abs(motionProfilePID.getLastError()) < MOTION_PROFILE_TOLERANCE);
    }
    public boolean checkZeroVelocity(){
        return stallDebouncer.calculate(Math.abs(motor1.getVelocity()) < 10);
    }
    public void resetElevatorStall(){
        setPower(0);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                new MotionState(getEncoderPosition(), getVelo(), 0),
                goalState,
                MAX_VELOCITY,
                MAX_ACCEL
        );
    }

    //TODO: Tune PID
    //call setUsePID(true) to use PID
    @Override
    public void periodic() {
        if (usePID) {
            if(useMotionProfiling){
                elapsedTime = System.currentTimeMillis() / (long)1000 - motionProfileStartTime;
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

        FtcDashboard.getInstance().getTelemetry().addData("position: ", getEncoderPosition());
        FtcDashboard.getInstance().getTelemetry().addData("getZeroVelo: ", checkZeroVelocity());
        FtcDashboard board = FtcDashboard.getInstance();
        board.getTelemetry().addData("loop time: ", deltaT);
        board.getTelemetry().addData("velo: ", getVelo());
        board.getTelemetry().addData("accel: ", accel);
        board.getTelemetry().addData("deltaV: ", deltaV);

        FtcDashboard.getInstance().getTelemetry().update();

    }
}
