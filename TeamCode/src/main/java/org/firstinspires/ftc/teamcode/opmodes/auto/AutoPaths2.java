package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// Field poses for CycleAuto2, in Pedro's coordinates: inches and degrees. Written for the blue side;
// these also run on red. The robot must really start at start, or the aiming will be off.
public class AutoPaths2 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(90, 135, 0);
    private final Pose path1 = poseFactory.of(132.0909, 134.9058, 0);
    private final Pose point2Start = poseFactory.of(132.0909, 134.9058, 0);
    private final Pose point2 = poseFactory.of(95.1118, 125.8846, 0);
    private final Pose point2Control1 = poseFactory.of(132.2041, 124.4193, 0);
    private final Pose point3 = poseFactory.of(117.184, 94.6539, 0);
    private final Pose point3Control1 = poseFactory.of(117.4587, 113.9785, 0);
    private final Pose point3Segment1Start = poseFactory.of(117.184, 94.6539, 0);
    private final Pose point3Segment1End = poseFactory.of(117.184, 94.6539, 0);
    private final Pose point4Start = poseFactory.of(117.184, 94.6539, 0);
    private final Pose point4 = poseFactory.of(127.5, 94.75, 0);
    private final Pose point5 = poseFactory.of(94.8371, 23.4917, -90);
    private final Pose point5Control1 = poseFactory.of(96.7603, 95.7529, 0);
    private final Pose point5Control2 = poseFactory.of(113.1542, 19.7367, 0);
    private final Pose point5Control3 = poseFactory.of(104.4535, 22.5759, 0);
    private final Pose point5Segment1Start = poseFactory.of(94.8371, 23.4917, 0);
    private final Pose point5Segment1End = poseFactory.of(94.8371, 23.4917, -90);
    private final Pose point6 = poseFactory.of(127.35, 48.9526, -90);
    private final Pose point6Control1 = poseFactory.of(112.3299, 22.3927, 0);
    private final Pose point6Control2 = poseFactory.of(109.0329, 32.9251, 0);
    private final Pose point6Control3 = poseFactory.of(87.2354, 52.4329, 0);
    private final Pose point6Segment1Start = poseFactory.of(127.35, 48.9526, -90);
    private final Pose point6Segment1End = poseFactory.of(127.35, 48.9526, -90);

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

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).heading(Interpolator.piecewise()
                .until(1, Interpolator.linear(point3Segment1Start, point3Segment1End)));
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4)
                .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.AUTO2_PATH4_SPEED));
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5Control2, point5Control3, point5).heading(Interpolator.piecewise()
                .until(1, Interpolator.linear(point5Segment1Start, point5Segment1End)));
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6Control2, point6Control3, point6).heading(Interpolator.piecewise()
                .until(1, Interpolator.linear(point6Segment1Start, point6Segment1End)));
    }
}
