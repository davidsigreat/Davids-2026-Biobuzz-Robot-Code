package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    // Degrees across servo positions 0 to 1: Axon MAX MK2 at 0.175 deg/us over the SDK's 600-2400 us range.
    private static final double SERVO_RANGE_DEGREES = 315;
    private static final double CENTER_POSITION = 0.5;
    private static final double MIN_ANGLE = -90;
    private static final double MAX_ANGLE = 90;

    private final Servo leftServo;
    private final Servo rightServo;
    private double targetAngle = 0;

    public Turret(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "turretLeft");
        rightServo = hardwareMap.get(Servo.class, "turretRight");
        // Both servos drive the same turret, so they must turn it the same way or they will fight.
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setAngle(double degrees) {
        targetAngle = Range.clip(degrees, MIN_ANGLE, MAX_ANGLE);
        double position = CENTER_POSITION + targetAngle / SERVO_RANGE_DEGREES;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
