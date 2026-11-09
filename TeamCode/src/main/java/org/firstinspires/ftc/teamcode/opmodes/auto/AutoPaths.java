package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// Field poses for CycleAuto, in Pedro's coordinates: inches and degrees. Written for the blue side;
// these also run on red. The robot must really start at start, or the aiming will be off.
public class AutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(103.5, 134, 0);
    private final Pose path1 = poseFactory.of(132.0909, 134.9058, 0);
    private final Pose point2Start = poseFactory.of(132.0909, 134.9058, 0);
    private final Pose point2 = poseFactory.of(95.1118, 125.8846, 0);
    private final Pose point2Control1 = poseFactory.of(132.2041, 124.4193, 0);

    public Pose start() {
        return start;
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1)
                .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.AUTO_PATH1_SPEED));
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
}
