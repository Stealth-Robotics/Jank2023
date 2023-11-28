package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final DcMotorEx motor3;
    //PID Constants
    //TODO: Tune PID
    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0.0000;
    private final double kF = 0;
    private boolean useMotionProfiling = false;
    private final PIDFController elevatorPID = new PIDFController(kP, kI, kD, kF);
    private final com.acmerobotics.roadrunner.control.PIDFController motionProfilePID =
            new com.acmerobotics.roadrunner.control.PIDFController(new PIDCoefficients(0, 0, 0));
    private final double SPEED_LIMIT = 0.25;

    //MOTION PROFILING CONSTANTS
    //VALUES ARE IN TICKS/SECOND (TICKS/SECOND^2 FOR ACCEL)
    private final double MAX_VELOCITY = 0.0;
    private final double MAX_ACCEL = 0.0;
    //TODO: TUNE TOLERANCE
    private final double MOTION_PROFILE_TOLERANCE = 10.0;

    private long motionProfileStartTime = 0;
    private long elapsedTime = 0;

    private MotionProfile profile;
    private boolean usePID = true;
    private double deltaT = 0;


    private final Debouncer stallDebouncer;


    private int level = 1;

    public enum ElevatorPosition {
        SCORE_POSITION(0.0),
        STOW_POSITION(0.0),

        LEVEL_ONE(650),
        LEVEL_TWO(790),
        LEVEL_THREE(820),

        LEVEL_FOUR(1150),

        LEVEL_FIVE(1380),
        LEVEL_SIX(1610),

        AUTO_SCORE(300);



        private final double value;

        ElevatorPosition(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        stallDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorPID.setTolerance(20);
        timer.start();


    }

    public void setSetpoint(double pos) {
        elevatorPID.setSetPoint(pos);
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetPoint();
    }


    public double getEncoderPosition() {
        return motor2.getCurrentPosition();
    }

    public void resetEncoderZero() {
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setToCurrentPosition() {
        elevatorPID.setSetPoint(getEncoderPosition());
    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
    }

    public void setSlowly() {
        stallDebouncer.calculate(false);
        setPower(-0.7);

    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public double getVelo() {
        return motor1.getVelocity();
    }

    public boolean motionProfilingAtSetpoint() {
        return (Math.abs(motionProfilePID.getLastError()) < MOTION_PROFILE_TOLERANCE);
    }

    public boolean checkZeroVelocity() {
        return stallDebouncer.calculate(Math.abs(motor2.getVelocity()) < 10);
    }

    public void resetElevatorStall() {
        setPower(0);
        resetEncoderZero();
    }

    public void incrementLevel(int increment){
        level += increment;
        if(level > 5){
            level = 5;
        }
        if(level < 1){
            level = 1;
        }

    }
    public int getLevel(){
        return level;
    }
    public void setUseMotionProfiling(boolean useMotionProfiling) {
        this.useMotionProfiling = useMotionProfiling;
    }

    //runs to position using motion profiling
    //generates motion profile based on current state and user specified goalState
    //also must take in start time for motion profile to profile
    public void runToPositionMotionProfiling(MotionState goalState, long timeStarted) {
        motionProfileStartTime = timeStarted / (long) 1000;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getEncoderPosition(), getVelo(), 0),
                goalState,
                MAX_VELOCITY,
                MAX_ACCEL
        );
    }

    //TODO: Tune PID
    //call setUsePID(true) to use PID
    Timing.Timer timer = new Timing.Timer(500);
    @Override
    public void periodic() {
        if (usePID) {
            if (useMotionProfiling) {
                elapsedTime = System.currentTimeMillis() / (long) 1000 - motionProfileStartTime;
                MotionState state = profile.get(elapsedTime);
                motionProfilePID.setTargetVelocity(state.getV());
                motionProfilePID.setTargetAcceleration(state.getA());
                motionProfilePID.setTargetPosition(state.getX());
                double power = motionProfilePID.update(getEncoderPosition());
                setPower(MathUtils.clamp(power, -SPEED_LIMIT, SPEED_LIMIT));
            }
            else {
                double power = MathUtils.clamp(elevatorPID.calculate(getEncoderPosition()), -1, 1);
                setPower(power);
            }
        }
        FtcDashboard.getInstance().getTelemetry().addData("elapsed time: ", timer.elapsedTime());
        timer.start();

        telemetry.addData("position: ", getEncoderPosition());
        FtcDashboard.getInstance().getTelemetry().addData("getZeroVelo: ", checkZeroVelocity());
//        FtcDashboard board = FtcDashboard.getInstance();
//        board.getTelemetry().addData("loop time: ", deltaT);
//
//
        FtcDashboard.getInstance().getTelemetry().addData("power", motor1.getPower());
        telemetry.addData("runpid", usePID);
        telemetry.addData("level", getLevel());
//
        telemetry.update();


    }
}
