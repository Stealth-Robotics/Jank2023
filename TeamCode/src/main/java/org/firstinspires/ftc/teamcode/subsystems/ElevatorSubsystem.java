package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    //TODO: Tune PID
    private final PIDFController elevatorPID = new PIDFController(0, 0, 0, 0);
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
            double power = elevatorPID.calculate(getEncoderPosition());
            setPower(power);
        }

    }
}
