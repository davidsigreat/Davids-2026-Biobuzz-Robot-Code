package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// One place for device names and robot settings. Pedro's tuner output stays in pedro/Constants.java.
//
// Names must match the Robot Configuration exactly, including capitals. Ports are chosen in the
// Robot Configuration, not in code, so the port comments are only a record of the wiring.
//
// Non-final values can be edited live from Panels. Those edits reset on restart, so copy keepers here.
@Configurable
public class RobotConstants {

    // Drivetrain
    public static final String FRONT_LEFT_NAME = "FL";   // Control Hub motor 0
    public static final String FRONT_RIGHT_NAME = "FR";  // Control Hub motor 1
    public static final String BACK_LEFT_NAME = "BL";    // Control Hub motor 2
    public static final String BACK_RIGHT_NAME = "BR";   // Control Hub motor 3
    public static final String PINPOINT_NAME = "PinPoint";  // Control Hub I2C bus 1

    // Flywheel
    public static final String FLYWHEEL_LEFT_NAME = "Shooter1";   // Expansion Hub motor 1 + encoder 1
    public static final String FLYWHEEL_RIGHT_NAME = "Shooter2";  // Expansion Hub motor 0
    public static final DcMotorSimple.Direction FLYWHEEL_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FLYWHEEL_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    // FLOAT so the wheel spins down on its own instead of braking against the motor.
    public static final DcMotor.ZeroPowerBehavior FLYWHEEL_ZERO_POWER = DcMotor.ZeroPowerBehavior.FLOAT;
    // goBILDA bare 6000 RPM motor, geared 1:1 to the wheel.
    public static final double FLYWHEEL_TICKS_PER_REV = 28;
    public static double FLYWHEEL_RPM_TOLERANCE = 50;
    public static double FLYWHEEL_SHOOT_RPM = 3000;
    // How long the stopper stays open per shot in Robot.shoot(). Starting guess until tested.
    public static double SHOOT_FEED_MS = 300;
    public static int AUTO_SHOT_COUNT = 3;
    // Motor power = kS + kV * targetRpm + kP * rpmError. kS and kV are from Flywheel SysId, kP is a guess.
    public static double FLYWHEEL_KS = 0.1004;
    public static double FLYWHEEL_KV = 0.0001795;
    public static double FLYWHEEL_KP = 0.0005;

    // Intake
    public static final String INTAKE1_NAME = "Intake1";  // Expansion Hub motor 2
    public static final String INTAKE2_NAME = "Intake2";  // Expansion Hub motor 3
    // Both motors share a gearbox, so they must turn it the same way, or they will fight.
    public static final DcMotorSimple.Direction INTAKE1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;
    public static double INTAKE_IN_PERCENT = 100;
    public static double INTAKE_OUT_PERCENT = -100;

    // Turret
    public static final String TURRET1_NAME = "Turret1";  // Control Hub servo 0
    public static final String TURRET2_NAME = "Turret2";  // Control Hub servo 2
    // Neutral, travel limits and inversion are set on each servo with the Axon Programmer MK2, so both
    // servos match here. They drive the same turret, so they must turn it the same way or they will fight.
    public static final Servo.Direction TURRET1_DIRECTION = Servo.Direction.FORWARD;
    public static final Servo.Direction TURRET2_DIRECTION = Servo.Direction.FORWARD;
    // Turret degrees across servo positions 0 to 1. Starting guess: Axon MAX MK2 at 0.175 deg/us over
    // the SDK's 600-2400 us range, geared 1:1. Measure this once the servos are programmed.
    public static double TURRET_SERVO_RANGE_DEGREES = 315;
    public static double TURRET_CENTER_POSITION = 0.5;
    public static double TURRET_MIN_ANGLE = -90;
    public static double TURRET_MAX_ANGLE = 90;
    public static double TURRET_STEP_DEGREES = 5;

    // Stopper
    public static final String STOPPER_NAME = "Stopper";  // Control Hub servo 1
    public static double STOPPER_OPEN_POSITION = 0.5;
    public static double STOPPER_CLOSED_POSITION = 0.0;

    // Match loader
    public static final String MATCH_LOADER_NAME = "MatchLoader";  // Control Hub servo 3
    public static double MATCH_LOADER_LOAD_POSITION = 0.735;
    public static double MATCH_LOADER_RETRACT_POSITION = 0.05;
}
