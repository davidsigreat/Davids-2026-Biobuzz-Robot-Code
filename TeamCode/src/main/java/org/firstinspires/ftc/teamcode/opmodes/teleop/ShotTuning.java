package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

// Collects shot data. Drive to a spot, change the RPM until shots score, then save the point.
// The saved points print as two lists, for refitting the RPM curve in RobotConstants.
// The saved list is lost when this OpMode stops, so copy it before pressing stop.
// Everything is on gamepad1. The turret tracks the closest goal, and the sticks drive:
//   A: flywheel on/off             dpad up/down: change the test RPM
//   right trigger (hold): shoot    left trigger (hold): intake
//   B: save this distance and RPM  X: remove the last saved point
//   back (share on PS): reset the pose, with the robot pushed square into a corner
@TeleOp(name = "Shot Tuning", group = "Test")
public class ShotTuning extends OpMode {

    private static final double RPM_STEP = 50;

    private Robot robot;
    private Command feed;
    private Command intakeIn;

    private final TreeMap<Long, Double> savedPoints = new TreeMap<>();
    private Long lastSaved = null;
    private double testRpm = RobotConstants.FLYWHEEL_SHOOT_RPM;
    private boolean flywheelOn = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        feed = robot.feedCommand();
        intakeIn = robot.intake.inCommand();
    }

    @Override
    public void start() {
        robot.home();
        robot.aimCommand(false).schedule();

        Commands.infinite(() -> robot.follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        )).requiring(robot.follower).schedule();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            testRpm += RPM_STEP;
        } else if (gamepad1.dpadDownWasPressed()) {
            testRpm = Math.max(testRpm - RPM_STEP, RPM_STEP);
        }

        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }

        if (flywheelOn) {
            robot.flywheel.setTargetRpm(testRpm);
        } else {
            robot.flywheel.stop();
        }

        if (gamepad1.rightTriggerWasPressed()) {
            feed.schedule();
        } else if (gamepad1.rightTriggerWasReleased()) {
            feed.cancel();
        }

        if (gamepad1.leftTriggerWasPressed()) {
            intakeIn.schedule();
        } else if (gamepad1.leftTriggerWasReleased()) {
            intakeIn.cancel();
        }

        if (gamepad1.bWasPressed()) {
            // Whole inches, so saving again at the same spot replaces that point.
            lastSaved = Math.round(robot.distanceToGoal());
            savedPoints.put(lastSaved, testRpm);
            gamepad1.rumble(200);
        }

        if (gamepad1.xWasPressed() && lastSaved != null) {
            savedPoints.remove(lastSaved);
            lastSaved = null;
        }

        if (gamepad1.backWasPressed()) {
            robot.resetPoseToCorner();
            gamepad1.rumble(300);
        }

        robot.update();

        telemetry.addData("Distance To Goal", "%.1f in", robot.distanceToGoal());
        telemetry.addData("Tracking Goal", robot.goalNumber());
        telemetry.addData("Goal In Turret Range", robot.goalInTurretRange());
        telemetry.addData("Test RPM", testRpm);
        telemetry.addData("Flywheel RPM", "%.0f", robot.flywheel.getRpm());
        telemetry.addData("Flywheel At Speed", robot.flywheel.atSpeed());
        telemetry.addData("Saved Points", savedPoints.size());
        telemetry.addData("Distances", tableLine(true));
        telemetry.addData("RPMs", tableLine(false));
    }

    // TreeMap keeps the points sorted by distance, which keeps the lists easy to read.
    private String tableLine(boolean distances) {
        StringBuilder line = new StringBuilder("{");
        for (Map.Entry<Long, Double> point : savedPoints.entrySet()) {
            if (line.length() > 1) {
                line.append(", ");
            }
            line.append(String.format(Locale.US, "%.0f", distances ? (double) point.getKey() : point.getValue()));
        }
        return line.append("}").toString();
    }
}
