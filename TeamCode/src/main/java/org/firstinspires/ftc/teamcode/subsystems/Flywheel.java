package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    // goBILDA bare 6000 RPM motor, geared 1:1 to the wheel.
    private static final double TICKS_PER_REV = 28;
    private static final double RPM_TOLERANCE = 50;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private double targetRpm = 0;

    public Flywheel(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setUp(leftMotor);
        setUp(rightMotor);
    }

    private void setUp(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // FLOAT so the wheel spins down on its own instead of braking against the motor.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        leftMotor.setVelocity(ticksPerSecond);
        rightMotor.setVelocity(ticksPerSecond);
    }

    public void stop() {
        targetRpm = 0;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getLeftRpm() {
        return leftMotor.getVelocity() * 60 / TICKS_PER_REV;
    }

    public double getRightRpm() {
        return rightMotor.getVelocity() * 60 / TICKS_PER_REV;
    }

    public double getRpm() {
        return (getLeftRpm() + getRightRpm()) / 2;
    }

    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) <= RPM_TOLERANCE;
    }
}
