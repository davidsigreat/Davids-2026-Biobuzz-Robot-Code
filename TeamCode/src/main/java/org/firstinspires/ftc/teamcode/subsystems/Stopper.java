package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    private static final double OPEN_POSITION = 0.5;
    private static final double CLOSED_POSITION = 0.0;

    private final Servo servo;

    public Stopper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "stopper");
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(CLOSED_POSITION);
    }
}
