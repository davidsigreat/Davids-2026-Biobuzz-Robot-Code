package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

// Fires the preloads, intakes along paths 1 and 2, and fires again. Then with the match loader down,
// intakes along paths 3 and 4 and the first half of path 5, where the match loader goes back up. Fires
// at the end of path 5, then everything stops and the robot drives path 6. The turret and flywheel
// track the goal from the robot's pose until that last stop.
@Autonomous(name = "Cycle Auto 2", preselectTeleOp = "Drive")
public class CycleAuto2 extends OpMode {

    // How far along path 5 the intake stops and the match loader retracts, as a fraction of its length.
    private static final double PATH5_INTAKE_STOP = 0.5;
    // How long to wait after the flywheel first reaches speed before the first shots.
    private static final double FIRST_SHOT_SETTLE_MS = 1000;
    // The follower reports a new path as complete until its first update, so wait that out.
    private static final double PATH_START_WAIT_MS = 100;

    private final AutoPaths2 paths = new AutoPaths2();

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.follower.setPose(paths.start());
        robot.home();
        Command aim = robot.aimCommand(true);
        aim.schedule();

        Groups.sequential(
                robot.flywheel.spinUpCommand(robot.shotRpm()),
                Commands.waitUntil(robot.flywheel::atSpeed),
                Commands.waitMs(FIRST_SHOT_SETTLE_MS),
                shoot(),
                Groups.deadline(
                        Groups.sequential(drive(paths.path1()), drive(paths.path2())),
                        robot.intake.inCommand()
                ),
                shoot(),
                Groups.deadline(
                        Groups.sequential(
                                drive(paths.path3()),
                                followFor(paths.path4(), RobotConstants.AUTO2_PATH4_TIME_MS),
                                startPath5UntilIntakeStop()
                        ),
                        robot.intake.inCommand(),
                        robot.matchLoader.loadCommand()
                ),
                Commands.waitUntil(() -> !robot.follower.isBusy()),
                shoot(),
                robot.flywheel.stopCommand(),
                Commands.instant(aim::cancel),
                drive(paths.path6())
        ).schedule();
    }

    private Command shoot() {
        return robot.shootCommand(RobotConstants.AUTO_SHOT_COUNT, RobotConstants.AUTO2_SHOOT_FEED_MS);
    }

    // Starts the path and finishes after ms, whether or not the robot has reached the end. If it gets
    // there sooner, it holds the end pose for the rest of the time.
    private Command followFor(Path path, double ms) {
        return Groups.sequential(
                Commands.instant(() -> robot.follower.follow(path)).requiring(robot.follower),
                Commands.waitMs(ms)
        );
    }

    // Starts path 5 and finishes at PATH5_INTAKE_STOP. The robot keeps following the path after this.
    private Command startPath5UntilIntakeStop() {
        return Groups.sequential(
                Commands.instant(() -> robot.follower.follow(paths.path5())).requiring(robot.follower),
                Commands.waitMs(PATH_START_WAIT_MS),
                Commands.waitUntil(() -> robot.follower.completion() >= PATH5_INTAKE_STOP)
        );
    }

    // Follows the path, then waits for the robot to settle on its end pose. The follow command alone
    // finishes 97.5% of the way along the path, while the robot is still moving. Pedro stops being busy
    // once settled, or after its 100 ms timeout, so this can't get stuck here.
    private Command drive(Path path) {
        return Groups.sequential(
                PedroCommands.follow(robot.follower, path).requiring(robot.follower),
                Commands.waitUntil(() -> !robot.follower.isBusy())
        );
    }

    @Override
    public void loop() {
        robot.update();

        Pose robotPose = robot.follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel RPM", robot.flywheel.getRpm());
        telemetry.addData("Flywheel Target RPM", robot.flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", robot.flywheel.atSpeed());
        telemetry.addData("Turret Target", robot.turret.getTargetAngle());
        telemetry.addData("Goal In Turret Range", robot.goalInTurretRange());
        telemetry.addData("Distance To Goal", robot.distanceToGoal());
    }
}
