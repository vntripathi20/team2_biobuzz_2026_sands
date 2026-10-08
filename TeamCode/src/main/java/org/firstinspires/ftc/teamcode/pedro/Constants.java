package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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
    });

    // ----------------------------------------------------------------------------------
    // 2. LOCALIZER  ->  run "Pinpoint Tuner", then paste its code here.
    // This assumes a goBILDA Pinpoint odometry computer. If you use something else
    // (OTOS, two/three dead wheels, OctoQuad), see the other tuners in pedro/procedures.
    // TODO: the offsets and directions below are placeholders until you run the tuner.
    // ----------------------------------------------------------------------------------
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ----------------------------------------------------------------------------------
    // 3. FORESIGHT  ->  run "Foresight Tuner", then paste its code here.
    // The robot CANNOT follow paths until this is filled in. These numbers depend on
    // your robot's weight, motors and wheels, so they must come from the tuner.
    // ----------------------------------------------------------------------------------
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // TODO: paste the Foresight Tuner output here.
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
