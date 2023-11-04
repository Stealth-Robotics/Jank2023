package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class ClawperSubsystem extends SubsystemBase {
    private final Servo clawServo;
    private final Servo rotationServo;


    private int positionCycle = 0;
    public static double TEST_POSITION = 0.45;


    public static enum ClawperPosition {
        ROTATION_STOW(0.45),
        ROTATION_SCORE(0.57),
        RELEASE_ONE(0.4),
        RELEASE_SECOND(0.57),
        RELEASE_BOTH(0.4),
        CLAW_HOLD(0.7),

        ROTATION_CLIMB(0.8);

        private final double position;

        ClawperPosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }

    public ClawperSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        rotationServo = hardwareMap.get(Servo.class, "rotator");
    }

    public void rotationToPosition(ClawperPosition position) {
        rotationServo.setPosition(position.getValue());
    }

    public void clawperToPosition(ClawperPosition position) {
        positionCycle = 1;
        clawServo.setPosition(position.getValue());
        positionCycle = 0;
    }

    public void clawperRelease() {
        //sets clawper to position based on whether we have 1 or 2 hexes
        if (positionCycle == 0) {
            clawServo.setPosition(ClawperPosition.RELEASE_ONE.getValue());
            positionCycle++;
            return;
        }
        if (positionCycle == 1) {
            clawServo.setPosition(ClawperPosition.RELEASE_SECOND.getValue());
            positionCycle = 0;
        }
    }

    public void clawperClosedPosition() {
        clawServo.setPosition(ClawperPosition.CLAW_HOLD.getValue());
        positionCycle = 0;
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("cycle: ", positionCycle);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
