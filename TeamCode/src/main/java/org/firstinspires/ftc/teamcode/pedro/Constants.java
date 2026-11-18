package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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

/**
 * ======================================================================================
 *                         PEDRO PATHING CONSTANTS (ROBOT SETTINGS)
 * ======================================================================================
 * Every Pedro Pathing OpMode builds its Follower from this file, so this is the ONE
 * place to describe your robot.
 *
 * The Follower is made of three parts:
 *   1. Drivetrain - the wheels/motors that move the robot          (drivetrainConfig)
 *   2. Localizer  - the sensor that tracks where the robot is      (localizerConfig)
 *   3. Algorithm  - "Foresight", the math that follows a path      (foresightConfig)
 *
 * HOW TO FILL THIS IN:
 *   Each config below has a matching tuner. Run the tuners IN ORDER (1 -> 2 -> 3).
 *   At the end, each tuner shows a block of Java code. Paste it over the matching
 *   config below, then re-deploy the app.
 *
 *   To open the tuners: connect your laptop to the robot's Wi-Fi, then open
 *   http://192.168.43.1:10158 in a web browser.
 * ======================================================================================
 */
public class Constants {

    // ----------------------------------------------------------------------------------
    // 1. DRIVETRAIN  ->  run "Mecanum Tuner", then paste its code here.
    // The names must match your Robot Configuration on the Driver Station EXACTLY.
    // ----------------------------------------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motor0");
        c.frontRightName.set("motor1");
        c.backLeftName.set("motor2");
        c.backRightName.set("motor3");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    // ----------------------------------------------------------------------------------
    // 2. LOCALIZER  ->  run "Pinpoint Tuner", then paste its code here.
    // This assumes a goBILDA Pinpoint odometry computer. If you use something else
    // (OTOS, two/three dead wheels, OctoQuad), see the other tuners in pedro/procedures.
    // Values copied from the team repo's prior localization AutoTune.
    // ----------------------------------------------------------------------------------
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        // The past-year config named the odometry computer "odo". It MUST match the name in
        // the Robot Configuration exactly (change this if yours says "pinpoint").
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.77558457382082);
        c.yPodOffset.set(-0.6094160230140987);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ----------------------------------------------------------------------------------
    // 3. FORESIGHT  ->  run "Foresight Tuner", then paste its code here.
    // ----------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Values copied from the team repo's prior Pedro AutoTune. Re-run the Foresight
        // Tuner if paths overshoot or wobble on this robot.
        Controller primaryForward = Controller.proportional(0.2700634096035219);
        Controller secondaryForward = Controller.proportional(0.09978121362129358);
        Controller primaryStrafe = Controller.proportional(1.8848664172851113);
        Controller secondaryStrafe = Controller.proportional(0.6964077766286013);

        c.forwardTranslational.set(Controller.piecewise(secondaryForward)
                .put(2.5, primaryForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryStrafe)
                .put(2.5, primaryStrafe));
        c.coast.set(Controller.proportionalFeedforward(0.009532781279950622));
        c.brake.set(Controller.proportionalFeedforward(0.00810286408795803));
        c.headingFeedback.set(Controller.proportional(6.888986279488746));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(
                0.05238218401060805, 0.006571389033228161));
        c.linearBrakeCoefficients.set(Matrix.diag(0.08167383157663453, 0.11921153780870264));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013993830534930764, 3.725465152997376E-4));
        c.maxAchievableForwardVelocity.set(79.94289256354337);
        c.maxAchievableStrafeVelocity.set(61.782452039412284);
        c.naturalForwardDeceleration.set(36.08066189407658);
        c.naturalStrafeDeceleration.set(207.02834921608405);
        c.maxVelocityConstraint.set(60.0);
        c.maxAccelerationConstraint.set(35.0);
        c.maxDecelerationConstraint.set(45.0);
    });

    // ----------------------------------------------------------------------------------
    // Factory methods - you should not need to change anything below this line.
    // ----------------------------------------------------------------------------------
    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm createAlgorithm() {
        try {
            return new Foresight(foresightConfig);
        } catch (IllegalStateException e) {
            throw new IllegalStateException(
                    "Foresight is not tuned yet! Run the Foresight Tuner and paste its code "
                            + "into Constants.foresightConfig.", e);
        }
    }

    /** Builds the Follower that drives the robot along paths. Call this once in init. */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
