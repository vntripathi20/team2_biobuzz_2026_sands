package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Every method marked with @Tuner shows up as a tuner on the tuning web page
 * (http://192.168.43.1:10158 while connected to the robot's Wi-Fi).
 *
 * Run them in this order: Mecanum Tuner -> Pinpoint Tuner -> Foresight Tuner -> Tests.
 * Using a different localizer? Swap PinpointTuner for the matching tuner in procedures/.
 */
public class Tuning {
    @Tuner(name = "1. Mecanum Tuner")
    static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "4. Tests")
    static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
