package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// Everything TeleOp and Auto share: the subsystems, the Ivy update loop, and commands that use more
// than one subsystem. Don't call setStart/setEnd/requiring on an Ivy group: it replaces the group's logic.
//
// All poses are written for the blue side. The field is the same when rotated 180 degrees, so a robot
// set up the same way on red sees the same thing around it, and the same code works with no alliance
// setting. On red, Panels shows the robot on the blue side.
public class Robot {
    // Static so it carries over from Auto into TeleOp.
    private static Pose lastPose = new Pose(FieldMath.FIELD_SIZE / 2, FieldMath.FIELD_SIZE / 2, 0);

    private boolean feeding = false;

    public final Follower follower;
    public final Intake intake;
    public final Flywheel flywheel;
    public final Turret turret;
    public final Stopper stopper;
    public final MatchLoader matchLoader;

    public Robot(HardwareMap hardwareMap) {
        // The scheduler is static, so commands from the last OpMode would otherwise still be in it.
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        // Creating the follower resets the Pinpoint, so restore where the last OpMode ended.
        follower.setPose(lastPose);
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        matchLoader = new MatchLoader(hardwareMap);
    }

    // Call from start(), not init(), so nothing moves before the match starts.
    public void home() {
        stopper.close();
        matchLoader.retract();
        turret.setAngle(0);
    }

    // Call once per loop.
    public void update() {
        Scheduler.execute();
        flywheel.update();
        follower.update();
        lastPose = follower.pose();
        showPoseOnPanels();
    }

    private Pose[] goals() {
        return new Pose[] {
                new Pose(RobotConstants.BLUE_GOAL_1_X, RobotConstants.BLUE_GOAL_1_Y, 0),
                new Pose(RobotConstants.BLUE_GOAL_2_X, RobotConstants.BLUE_GOAL_2_Y, 0)
        };
    }

    private int goalIndex() {
        return follower.pose().y() > RobotConstants.GOAL_SPLIT_Y ? 0 : 1;
    }

    private Pose goal() {
        return goals()[goalIndex()];
    }

    // 1 or 2, matching the names in RobotConstants.
    public int goalNumber() {
        return goalIndex() + 1;
    }

    private Pose turretPose() {
        return FieldMath.turretPose(follower.pose(), RobotConstants.TURRET_FORWARD_OFFSET_INCHES);
    }

    // From the turret, since that's where shots leave the robot.
    public double distanceToGoal() {
        return FieldMath.distance(turretPose(), goal());
    }

    // Flywheel RPM for the robot's current distance from the goal.
    public double shotRpm() {
        return ((RobotConstants.SHOT_RPM_BASE * Math.pow(RobotConstants.SHOT_RPM_GROWTH, distanceToGoal()))*RobotConstants.SHOT_RPM_MULTIPLIER);
    }

    public boolean farEnoughToShoot() {
        return distanceToGoal() > RobotConstants.MIN_SHOT_DISTANCE_INCHES;
    }

    // Turret angle that faces the goal, which can be past what the turret can reach.
    public double angleToGoal() {
        Pose goal = goal();
        return FieldMath.turretAngle(turretPose(), goal.x(), goal.y());
    }

    public boolean goalInTurretRange() {
        double angle = angleToGoal();
        return angle >= RobotConstants.TURRET_MIN_ANGLE && angle <= RobotConstants.TURRET_MAX_ANGLE;
    }

    // Call with the robot pushed square into any field corner.
    public void resetPoseToCorner() {
        follower.setPose(FieldMath.cornerPose(follower.pose(), RobotConstants.ROBOT_CENTER_TO_FRONT_INCHES,
                RobotConstants.ROBOT_CENTER_TO_BACK_INCHES, RobotConstants.ROBOT_WIDTH_INCHES));
    }

    // Keeps the turret on the closest goal. With useShotRpm, it also keeps the flywheel at the right
    // RPM for that distance while it's on. Doesn't require the flywheel, so turning it on and off doesn't
    // stop the aiming.
    public Command aimCommand(boolean useShotRpm) {
        return Commands.infinite(() -> {
            turret.setAngle(angleToGoal());
            if (useShotRpm && flywheel.getTargetRpm() > 0) {
                flywheel.setTargetRpm(shotRpm());
            }
        }).requiring(turret);
    }

    // Draws the robot at its real size on the Panels field, from the odometry pose. Panels rectangles
    // can't rotate, so this draws the four sides as lines. The front side is white.
    private void showPoseOnPanels() {
        Pose pose = follower.pose();
        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();
        double[][] corners = FieldMath.robotCorners(pose, RobotConstants.ROBOT_CENTER_TO_FRONT_INCHES,
                RobotConstants.ROBOT_CENTER_TO_BACK_INCHES, RobotConstants.ROBOT_WIDTH_INCHES);

        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        for (int i = 0; i < corners.length; i++) {
            String color = i == 0 ? PanelsField.INSTANCE.getWHITE() : PanelsField.INSTANCE.getBLUE();
            double[] next = corners[(i + 1) % corners.length];
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 1);
            field.moveCursor(corners[i][0], corners[i][1]);
            field.line(next[0], next[1]);
        }

        // The goal being tracked is red, and the other one is black.
        Pose[] goals = goals();
        for (int i = 0; i < goals.length; i++) {
            String color = i == goalIndex() ? PanelsField.INSTANCE.getRED() : PanelsField.INSTANCE.getBLACK();
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 1);
            field.moveCursor(goals[i].x(), goals[i].y());
            field.circle(3);
        }
        field.update();

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.addData("Robot X", x);
        panelsTelemetry.addData("Robot Y", y);
        panelsTelemetry.addData("Robot Heading", Math.toDegrees(heading));
        panelsTelemetry.update();
    }

    // Waits for the flywheel to reach speed once, then keeps the stopper open and the intake running
    // until cancelled, even if the speed drops between shots. Pauses while too close to the goal, then
    // waits for speed again.
    public Command feedCommand() {
        return Command.build()
                .setStart(() -> feeding = false)
                .setExecute(this::runFeed)
                .setEnd(end -> stopFeed())
                .requiring(stopper, intake);
    }

    private void runFeed() {
        if (!farEnoughToShoot()) {
            feeding = false;
        } else if (flywheel.atSpeed()) {
            feeding = true;
        }

        if (feeding) {
            stopper.open();
            intake.shoot();
        } else {
            stopFeed();
        }
    }

    private void stopFeed() {
        stopper.close();
        intake.stop();
    }

    // Fires one game piece each time the flywheel is at speed and the robot is far enough from the goal.
    // The flywheel must already be spinning and the robot past the minimum distance, or this waits
    // forever. Runs the intake to push game pieces up to the stopper.
    public Command shootCommand(int shots) {
        return shootCommand(shots, RobotConstants.SHOOT_FEED_MS);
    }

    // Same, with the stopper open for feedMs per shot.
    public Command shootCommand(int shots, double feedMs) {
        Command oneShot = Groups.sequential(
                Commands.waitUntil(() -> flywheel.atSpeed() && farEnoughToShoot()),
                Groups.deadline(Commands.waitMs(feedMs), stopper.openCommand())
        );
        return Groups.deadline(Groups.repeat(oneShot, shots), intake.shootCommand());
    }
}
