package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawperSubsystem extends SubsystemBase {
    private final Servo clawServo;
    private final Servo rotationServo;

    //POSITIONS
    private final double ROTATION_STOW = 0.0;
    private final double ROTATION_SCORE = 0.0;
    private final double RELEASE_ONE = 0.0;
    private final double RELEASE_SECOND = 0.0;
    private final double RELEASE_BOTH = 0.0;
    private final double CLAW_HOLD = 0.0;

    private int positionCycle = 0;

    public ClawperSubsystem(HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, "claw");
        rotationServo = hardwareMap.get(Servo.class, "rotator");
    }

    public void rotationToPosition(double position){
        rotationServo.setPosition(position);
    }

    public void clawperRelease(){
        if(positionCycle == 0){
            clawServo.setPosition(RELEASE_ONE);
            positionCycle++;
        }
        if(positionCycle == 1){
            clawServo.setPosition(RELEASE_SECOND);
            positionCycle = 0;
        }
    }

    public void clawperClosedPosition(){
        clawServo.setPosition(CLAW_HOLD);
    }

}
