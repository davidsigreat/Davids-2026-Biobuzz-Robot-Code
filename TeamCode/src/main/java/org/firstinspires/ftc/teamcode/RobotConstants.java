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
    // For the corner pose reset and the Panels drawing. Front and back are measured from the center of rotation, which is
    // behind the middle of the robot. Width is side to side.
    public static double ROBOT_CENTER_TO_FRONT_INCHES = 10;
    public static double ROBOT_CENTER_TO_BACK_INCHES = 7.625;
    public static double ROBOT_WIDTH_INCHES = 13;

    /**
     * Aiming
     * Blue's two seesaw goals in Pedro's coordinates. These work on red too, because the field is the
     * same when rotated 180 degrees.
     */
    public static double BLUE_GOAL_1_X = 84.0;
    public static double BLUE_GOAL_1_Y = 89.5;
    public static double BLUE_GOAL_2_X = 84.0;
    public static double BLUE_GOAL_2_Y = 52.0;

    // The robot tracks goal 1 above this y and goal 2 at or below it. The middle of the field.
    public static double GOAL_SPLIT_Y = 70.75; //Middle

    // Flywheel RPM at a distance from the goal in inches: SHOT_RPM_BASE * SHOT_RPM_GROWTH ^ distance.
    // Fitted to tested shots.
    public static double SHOT_RPM_BASE = 2629.33874;
    public static double SHOT_RPM_GROWTH = 1.00388;
    // The robot won't shoot this close to the goal, or closer.
    public static double MIN_SHOT_DISTANCE_INCHES = 24;

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
    // Motor power = kS + kV * rpm + kA * rpmPerSecond + kP * rpmError + kD * rpmErrorPerSecond.
    // Copy these from Flywheel SysId. kA and kD do nothing until they are set.
    public static double FLYWHEEL_KS = 0.0735;
    public static double FLYWHEEL_KV = 0.0001776;
    public static double FLYWHEEL_KA = 0.0001457;
    public static double FLYWHEEL_KP = 0.001;
    public static double FLYWHEEL_KD = 0.0000022;
    // How fast the loop's setpoint climbs to the target, which is what gives kA something to act on.
    public static double FLYWHEEL_RAMP_RPM_PER_SECOND = 2702;
    // Smooths the error rate for kD, since the 28 tick encoder is noisy. 1 is no smoothing.
    public static double FLYWHEEL_D_FILTER = 0.3;

    // Intake
    public static final String INTAKE1_NAME = "Intake1";  // Expansion Hub motor 2
    public static final String INTAKE2_NAME = "Intake2";  // Expansion Hub motor 3
    // Both motors share a gearbox, so they must turn it the same way, or they will fight.
    public static final DcMotorSimple.Direction INTAKE1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;
    public static double INTAKE_IN_PERCENT = 100;
    public static double INTAKE_OUT_PERCENT = -100;
    public static double INTAKE_SHOOT_PERCENT = 80;

    // Turret
    public static final String TURRET1_NAME = "Turret1";  // Control Hub servo 0
    public static final String TURRET2_NAME = "Turret2";  // Control Hub servo 2
    // Neutral, travel limits and inversion are set on each servo with the Axon Programmer MK2, so both
    // servos match here. They drive the same turret, so they must turn it the same way or they will fight.
    public static final Servo.Direction TURRET1_DIRECTION = Servo.Direction.FORWARD;
    public static final Servo.Direction TURRET2_DIRECTION = Servo.Direction.FORWARD;
    // The Axon MAX MK2's full signal range, wider than the SDK's default 600-2400 us.
    public static final double TURRET_PWM_MIN_US = 500;
    public static final double TURRET_PWM_MAX_US = 2500;
    // Turret degrees across servo positions 0 to 1. Starting guess: 0.175 deg/us over the 2000 us
    // signal range, geared 1:1. Measure this once the servos are programmed.
    public static double TURRET_SERVO_RANGE_DEGREES = 350;
    public static double TURRET_CENTER_POSITION = 0.5;
    // 0 is the back of the robot and + is clockwise seen from above. The turret can reach 180 each way,
    // so these stop just short of that.
    public static double TURRET_MIN_ANGLE = -175;
    public static double TURRET_MAX_ANGLE = 175;
    public static double TURRET_STEP_DEGREES = 5;

    // Stopper
    public static final String STOPPER_NAME = "Stopper";  // Control Hub servo 1
    public static double STOPPER_OPEN_POSITION = 0.6;
    public static double STOPPER_CLOSED_POSITION = 0.35;

    // Match loader
    public static final String MATCH_LOADER_NAME = "MatchLoader";  // Control Hub servo 3
    public static double MATCH_LOADER_LOAD_POSITION = 0.735;
    public static double MATCH_LOADER_RETRACT_POSITION = 0.05;
}
