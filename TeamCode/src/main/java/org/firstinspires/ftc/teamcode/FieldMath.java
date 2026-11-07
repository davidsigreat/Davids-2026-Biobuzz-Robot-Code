package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

// Field geometry in Pedro's coordinates: inches, 0 to FIELD_SIZE on each axis, heading in radians.
// Takes everything as arguments, so it can be tested without a robot.
public class FieldMath {
    // The real field is smaller than the nominal 144 inches.
    public static final double FIELD_SIZE = 141.5;
    private static final double QUARTER_TURN = Math.PI / 2;

    public static double distance(Pose a, Pose b) {
        return Math.hypot(b.x() - a.x(), b.y() - a.y());
    }

    // Turret angle that faces the point, from -180 to 180. The turret's zero is the back of the robot,
    // and + is clockwise seen from above.
    public static double turretAngle(Pose robot, double pointX, double pointY) {
        double fieldAngle = Math.atan2(pointY - robot.y(), pointX - robot.x());
        double counterclockwiseFromFront = Math.toDegrees(fieldAngle - robot.heading());
        double degrees = 180 - counterclockwiseFromFront;
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }

    // The robot's four corners as {x, y}: front left, front right, back right, back left.
    public static double[][] robotCorners(Pose pose, double toFront, double toBack, double robotWidth) {
        double halfWidth = robotWidth / 2;
        return new double[][] {
                offset(pose, toFront, halfWidth),
                offset(pose, toFront, -halfWidth),
                offset(pose, -toBack, -halfWidth),
                offset(pose, -toBack, halfWidth)
        };
    }

    // The point this far ahead of and to the left of the pose, in field coordinates.
    private static double[] offset(Pose pose, double forward, double left) {
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        return new double[] {pose.x() + forward * cos - left * sin, pose.y() + forward * sin + left * cos};
    }

    // The exact pose of a robot pushed square into the corner nearest its current pose, with its
    // heading snapped to the nearest quarter turn. toFront and toBack are the distances from the
    // robot's center of rotation to its front and back bumpers, which don't have to match.
    public static Pose cornerPose(Pose current, double toFront, double toBack, double robotWidth) {
        long quarterTurns = Math.round(current.heading() / QUARTER_TURN);
        long direction = Math.floorMod(quarterTurns, 4L);
        boolean facingAlongX = direction % 2 == 0;
        // Facing +x or +y, so the front is toward the high side of the field.
        boolean facingHighSide = direction <= 1;
        double toHighSide = facingHighSide ? toFront : toBack;
        double toLowSide = facingHighSide ? toBack : toFront;
        double halfWidth = robotWidth / 2;

        double x = facingAlongX
                ? againstWall(current.x(), toLowSide, toHighSide)
                : againstWall(current.x(), halfWidth, halfWidth);
        double y = facingAlongX
                ? againstWall(current.y(), halfWidth, halfWidth)
                : againstWall(current.y(), toLowSide, toHighSide);
        return new Pose(x, y, quarterTurns * QUARTER_TURN);
    }

    // Where the robot's center sits when touching the wall nearest to position.
    private static double againstWall(double position, double toLowSide, double toHighSide) {
        return position < FIELD_SIZE / 2 ? toLowSide : FIELD_SIZE - toHighSide;
    }
}
