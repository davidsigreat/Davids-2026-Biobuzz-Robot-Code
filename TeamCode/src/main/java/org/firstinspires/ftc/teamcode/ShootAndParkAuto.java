package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Drives to SHOOT_POSE while the flywheel spins up, fires the preloads, then drives to PARK_POSE.
@Autonomous(name = "Shoot And Park")
public class ShootAndParkAuto extends OpMode {

    // Placeholders measured from wherever the robot starts, in inches and radians.
    // Replace with real field poses before using this at a match.
    private static final Pose START_POSE = new Pose(0, 0, 0);
    private static final Pose SHOOT_POSE = new Pose(24, 0, 0);
    private static final Pose PARK_POSE = new Pose(24, 24, 0);

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setPose(START_POSE);
    }

    @Override
    public void start() {
        robot.home();

        Path toShoot = line(START_POSE, SHOOT_POSE).constant(SHOOT_POSE.heading());
        Path toPark = line(SHOOT_POSE, PARK_POSE).constant(PARK_POSE.heading());

        Groups.sequential(
                Groups.parallel(
                        PedroCommands.follow(robot.follower, toShoot).requiring(robot.follower),
                        robot.flywheel.spinUpCommand(RobotConstants.FLYWHEEL_SHOOT_RPM)
                ),
                robot.shootCommand(RobotConstants.AUTO_SHOT_COUNT),
                robot.flywheel.stopCommand(),
                PedroCommands.follow(robot.follower, toPark).requiring(robot.follower)
        ).schedule();
    }

    @Override
    public void loop() {
        robot.update();

        Pose robotPose = robot.follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel RPM", robot.flywheel.getRpm());
        telemetry.addData("Flywheel At Speed", robot.flywheel.atSpeed());
    }
}
