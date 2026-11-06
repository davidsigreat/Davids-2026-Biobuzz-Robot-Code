package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MatchLoader {
    private static final double LOAD_POSITION = 0.5;
    private static final double RETRACT_POSITION = 0.0;

    private final Servo servo;

    public MatchLoader(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "matchLoader");
    }

    public void load() {
        servo.setPosition(LOAD_POSITION);
    }

    public void retract() {
        servo.setPosition(RETRACT_POSITION);
    }
}
