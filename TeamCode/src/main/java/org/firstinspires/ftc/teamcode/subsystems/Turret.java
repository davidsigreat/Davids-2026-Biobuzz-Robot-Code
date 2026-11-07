package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Turret {
    private final Servo turret1;
    private final Servo turret2;
    private double targetAngle = 0;

    public Turret(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(Servo.class, RobotConstants.TURRET1_NAME);
        turret2 = hardwareMap.get(Servo.class, RobotConstants.TURRET2_NAME);
        turret1.setDirection(RobotConstants.TURRET1_DIRECTION);
        turret2.setDirection(RobotConstants.TURRET2_DIRECTION);
    }

    public void setAngle(double degrees) {
        targetAngle = Range.clip(degrees, RobotConstants.TURRET_MIN_ANGLE, RobotConstants.TURRET_MAX_ANGLE);
        double position = RobotConstants.TURRET_CENTER_POSITION + targetAngle / RobotConstants.TURRET_SERVO_RANGE_DEGREES;
        turret1.setPosition(position);
        turret2.setPosition(position);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public Command stepCommand(double degrees) {
        return Commands.instant(() -> setAngle(targetAngle + degrees)).requiring(this);
    }

    public Command centerCommand() {
        return Commands.instant(() -> setAngle(0)).requiring(this);
    }
}
