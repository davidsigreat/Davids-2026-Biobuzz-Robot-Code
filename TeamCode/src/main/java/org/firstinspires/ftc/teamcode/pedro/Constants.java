package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Constants {
    // Replace with the Mecanum Tuner output (directions below are a starting guess)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConstants.FRONT_LEFT_NAME);
        c.frontRightName.set(RobotConstants.FRONT_RIGHT_NAME);
        c.backLeftName.set(RobotConstants.BACK_LEFT_NAME);
        c.backRightName.set(RobotConstants.BACK_RIGHT_NAME);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        //c.manualBrakeMode.set(true);
    });



    public static PinpointConfig localizerConfig = new PinpointConfig(
        c -> {
            c.name.set(RobotConstants.PINPOINT_NAME);
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(1.738704);
            c.yPodOffset.set(-4.471457);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        }
    );

    // Replace with the Foresight Tuner output. Until then create() throws "Config variable has not been set".
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3294912684229801);
                Controller secondaryTranslationalForward = Controller.proportional(0.1217382195134502);
                Controller primaryTranslationalLateral = Controller.proportional(0.7365229191067113);
                Controller secondaryTranslationalLateral = Controller.proportional(0.27212553835507486);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013664983214688358));
                c.brake.set(Controller.proportionalFeedforward(0.011615235732485104));

                c.headingFeedback.set(Controller.proportional(5.893069929034888));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05938278093465365, 0.004598357921665435));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10274519550456734, 0.03725815643563329));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.91992462317978E-4, 0.0025886202260079986));

                c.maxAchievableForwardVelocity.set(71.85814126285545);
                c.maxAchievableStrafeVelocity.set(53.113904463812275);
                c.naturalForwardDeceleration.set(34.96836816540898);
                c.naturalStrafeDeceleration.set(77.77295494351968);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
