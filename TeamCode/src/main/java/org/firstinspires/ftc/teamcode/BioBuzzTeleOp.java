package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;

/**
 * ======================================================================================
 *                       BIOBUZZ TELEOP (Sense and Sound Robotics)
 * ======================================================================================
 * Mecanum drive + intake + shooter that keeps itself pointed at the HIVE.
 *
 *   AUTO-AIM:  The Limelight (pipeline 0 = AprilTags) looks for the hive tags of your
 *              alliance. While you drive, the robot turns itself to keep the hive centered
 *              (you keep control of forward/strafe). Push the right stick to take over
 *              turning; let go and it re-aims.
 *   SHOOTER:   The flywheel RPM is picked from the distance to the hive tag using the
 *              SHOOTER_TABLE below. RT only feeds a game piece once the flywheels are up
 *              to speed.
 *   INTAKE:    Two stages. Outer = motor1B (picks pollen off the floor), inner = motor0B
 *              (moves it toward the shooter).
 *
 * INIT (gamepad 1):  X = BLUE alliance,  B = RED alliance
 *
 * CONTROLS (gamepad 1):
 *   Left stick        drive / strafe
 *   Right stick X     turn (overrides auto-aim while pushed)
 *   Right bumper      toggle auto-aim ON / OFF   (starts ON)
 *   Left bumper       toggle shooter spin-up ON / OFF (starts OFF)
 *   Right trigger     SHOOT (feeds when shooter is at speed)
 *   Left trigger      INTAKE (outer + inner)
 *   B                 reverse intake (spit out jams)
 *   D-pad left        AprilTag pipeline (aiming)
 *   D-pad right       Pollen pipeline (see what the model detects; auto-aim pauses)
 *
 * HIVE TAGS:  BLUE = IDs 38-45,  RED = IDs 30-37 (both sides of each hive).
 *
 * BEFORE THIS WILL RUN ON THE ROBOT:
 *   - Shooter motors (motor2B, motor3B) need their encoders plugged in, because the
 *     flywheel runs on velocity control.
 *   - Check SHOOTER_TICKS_PER_REV, the motor directions, and every "TODO" number below.
 * ======================================================================================
 */
@TeleOp(name = "BioBuzz TeleOp", group = "TeleOp")
public class BioBuzzTeleOp extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // 1. SETTINGS - tune these on your robot.
    // ----------------------------------------------------------------------------------

    // --- Limelight ---
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String LIMELIGHT_HOST = "172.29.0.1";
    private static final int LIMELIGHT_STREAM_PORT = 5800;
    private static final int APRILTAG_PIPELINE = 0;
    private static final int POLLEN_PIPELINE = 1;      // TODO: set to your pollen model's pipeline

    // --- Hive tag IDs (both sides of each hive) ---
    private static final int BLUE_HIVE_MIN_ID = 38;
    private static final int BLUE_HIVE_MAX_ID = 45;
    private static final int RED_HIVE_MIN_ID = 30;
    private static final int RED_HIVE_MAX_ID = 37;

    // --- Auto-aim (same convention as LimelightTeleOp) ---
    // AIM_DIRECTION: set to 1.0 if the robot turns toward the hive, -1.0 if it turns away.
    private static final double AIM_DIRECTION = -1.0;
    private static final double AIM_KP = 0.02;
    private static final double AIM_MIN_POWER = 0.05;
    private static final double AIM_MAX_POWER = 0.5;
    private static final double AIM_TOLERANCE_DEG = 1.5;
    // The tx you want to hold. The shooter is not mounted where the camera is, so this is
    // probably NOT zero. Aim the shooter at the hive by hand, read tx, and enter it here.
    private static final double AIM_TARGET_TX_DEG = 0.0;   // TODO
    private static final double TX_SMOOTHING = 0.5;
    // If the tag drops out, keep turning toward where it was last seen for this long.
    private static final double LOST_SEARCH_SECONDS = 0.5;
    private static final double LOST_SEARCH_POWER = 0.15;
    // Right stick beyond this takes over turning from auto-aim.
    private static final double AIM_DRIVER_OVERRIDE = 0.25;

    // --- Shooter ---
    private static final String INNER_SHOOTER_NAME = "motor2B";
    private static final String OUTER_SHOOTER_NAME = "motor3B";
    // TODO: flywheels usually spin opposite ways. Flip one if a wheel spins backwards.
    private static final DcMotorSimple.Direction INNER_SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction OUTER_SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    // Encoder ticks per flywheel revolution (motor CPR x any gear ratio to the wheel).
    // TODO: bare goBILDA 5203 = 28. Look up your exact motor and ratio.
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    // RPM used before any hive tag has been seen.
    private static final double DEFAULT_SHOOTER_RPM = 3000.0;   // TODO
    // "At speed" means both wheels are within this many RPM of the target.
    private static final double SHOOTER_RPM_TOLERANCE = 100.0;
    // Distance from camera to hive tag (meters) -> flywheel RPM. Must be sorted by distance.
    // TODO: these are placeholders. Shoot from several distances and record what works.
    private static final double[][] SHOOTER_TABLE = {
            {1.0, 2800.0},
            {1.5, 3200.0},
            {2.0, 3600.0},
            {2.5, 4000.0},
            {3.0, 4400.0},
    };
    private static final double DISTANCE_SMOOTHING = 0.3;

    // --- Intake ---
    private static final String INNER_INTAKE_NAME = "motor0B";
    private static final String OUTER_INTAKE_NAME = "motor1B";
    // TODO: flip a direction if that stage pushes pollen the wrong way.
    private static final DcMotorSimple.Direction INNER_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction OUTER_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_POWER = 1.0;   // inner stage power while shooting

    private static final double TRIGGER_THRESHOLD = 0.3;

    // ----------------------------------------------------------------------------------
    // 2. HARDWARE AND STATE
    // ----------------------------------------------------------------------------------
    private Limelight3A limelight;
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotorEx innerShooter, outerShooter;
    private DcMotor innerIntake, outerIntake;

    private final LimelightTeleOp.LimelightStream stream =
            new LimelightTeleOp.LimelightStream(LIMELIGHT_HOST, LIMELIGHT_STREAM_PORT);

    private boolean redAlliance = false;
    private int pipeline = APRILTAG_PIPELINE;

    private boolean aimEnabled = true;
    private boolean shooterEnabled = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean wasReady = false;

    // Hive tracking
    private final ElapsedTime sinceSeen = new ElapsedTime();
    private boolean haveHive = false;
    private double smoothedTx = 0.0;
    private double lastError = 0.0;
    private boolean haveDistance = false;
    private double smoothedDistance = 0.0;
    private double targetRpm = DEFAULT_SHOOTER_RPM;

    @Override
    public void runOpMode() {
        initDrive();
        initMechanisms();

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        stream.start();
        CameraStreamServer.getInstance().setSource(stream);

        // INIT: pick the alliance so we only aim at OUR hive tags.
        while (opModeInInit()) {
            if (gamepad1.x) redAlliance = false;
            if (gamepad1.b) redAlliance = true;
            telemetry.addLine("Press X for BLUE, B for RED, then PLAY");
            telemetry.addData("Alliance", redAlliance ? "RED (tags 30-37)" : "BLUE (tags 38-45)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleToggles();
            handlePipelineButtons();

            LLResult result = limelight.getLatestResult();
            LLResultTypes.FiducialResult hive = null;
            if (pipeline == APRILTAG_PIPELINE && result != null && result.isValid()) {
                hive = pickHiveTag(result);
            }

            // ------------------------------------------------------------------------------
            // DRIVING (+ auto-aim)
            // ------------------------------------------------------------------------------
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;
            double turn = gamepad1.right_stick_x;
            boolean driverTurning = Math.abs(gamepad1.right_stick_x) > AIM_DRIVER_OVERRIDE;

            if (hive != null) {
                updateHiveTracking(hive);
            }

            boolean aiming = false;
            if (aimEnabled && !driverTurning && pipeline == APRILTAG_PIPELINE) {
                if (hive != null) {
                    turn = aimTurnPower(lastError);
                    aiming = true;
                } else if (haveHive && sinceSeen.seconds() < LOST_SEARCH_SECONDS) {
                    turn = AIM_DIRECTION * Math.copySign(LOST_SEARCH_POWER, lastError);
                    aiming = true;
                }
            }
            drive(forward, strafe, turn);

            // ------------------------------------------------------------------------------
            // SHOOTER + INTAKE
            // ------------------------------------------------------------------------------
            updateShooter();
            boolean atSpeed = shooterAtSpeed();
            boolean aimed = haveHive && sinceSeen.seconds() < LOST_SEARCH_SECONDS
                    && Math.abs(lastError) < AIM_TOLERANCE_DEG;
            boolean readyToShoot = shooterEnabled && atSpeed && aimed;

            // Rumble once each time we become ready, so the driver can feel it.
            if (readyToShoot && !wasReady) gamepad1.rumble(150);
            wasReady = readyToShoot;

            updateIntake(atSpeed);

            // ------------------------------------------------------------------------------
            // TELEMETRY
            // ------------------------------------------------------------------------------
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("Pipeline", pipeline == APRILTAG_PIPELINE ? "AprilTag" : "Pollen");
            telemetry.addData("Auto-aim", !aimEnabled ? "OFF"
                    : driverTurning ? "paused (driver turning)"
                    : aiming ? "AIMING" : "no hive tag");
            telemetry.addData("Hive tx error", haveHive ? String.format("%.1f deg", lastError) : "-");
            telemetry.addData("Hive distance", haveDistance ? String.format("%.2f m", smoothedDistance) : "-");
            telemetry.addData("Shooter", shooterEnabled ? "SPINNING" : "off");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Inner / Outer RPM", "%.0f / %.0f", rpmOf(innerShooter), rpmOf(outerShooter));
            telemetry.addData("READY TO SHOOT", readyToShoot ? "YES" : "no");
            if (pipeline == POLLEN_PIPELINE && result != null && result.isValid()) {
                telemetry.addData("Pollen tx / ta", "%.1f deg / %.1f %%", result.getTx(), result.getTa());
            }
            telemetry.addData("Camera feed", stream.isStreaming()
                    ? "live - Driver Station menu > Camera Stream"
                    : "not connected: " + stream.getError());
            telemetry.update();
        }

        // Stop everything when the OpMode ends.
        drive(0, 0, 0);
        setShooterRpm(0);
        innerIntake.setPower(0);
        outerIntake.setPower(0);
        stream.stop();
        CameraStreamServer.getInstance().setSource(null);
        limelight.stop();
    }

    // ----------------------------------------------------------------------------------
    // 3. HIVE TAG TRACKING
    // ----------------------------------------------------------------------------------
    private boolean isHiveTag(int id) {
        return redAlliance
                ? id >= RED_HIVE_MIN_ID && id <= RED_HIVE_MAX_ID
                : id >= BLUE_HIVE_MIN_ID && id <= BLUE_HIVE_MAX_ID;
    }

    /** Of our alliance's hive tags in view, returns the one closest to the crosshair. */
    private LLResultTypes.FiducialResult pickHiveTag(LLResult result) {
        LLResultTypes.FiducialResult best = null;
        double bestOffset = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!isHiveTag(tag.getFiducialId())) continue;
            double offset = Math.abs(tag.getTargetXDegrees() - AIM_TARGET_TX_DEG);
            if (offset < bestOffset) {
                bestOffset = offset;
                best = tag;
            }
        }
        return best;
    }

    /** Smooths tx (for aiming) and the camera-to-tag distance (for flywheel RPM). */
    private void updateHiveTracking(LLResultTypes.FiducialResult hive) {
        double tx = hive.getTargetXDegrees();
        smoothedTx = haveHive ? TX_SMOOTHING * tx + (1 - TX_SMOOTHING) * smoothedTx : tx;
        haveHive = true;
        lastError = smoothedTx - AIM_TARGET_TX_DEG;
        sinceSeen.reset();

        // Camera-space pose: x = right, z = forward (meters). Flat distance to the tag.
        Pose3D pose = hive.getTargetPoseCameraSpace();
        if (pose != null) {
            double distance = Math.hypot(pose.getPosition().x, pose.getPosition().z);
            if (distance > 0.0) {
                smoothedDistance = haveDistance
                        ? DISTANCE_SMOOTHING * distance + (1 - DISTANCE_SMOOTHING) * smoothedDistance
                        : distance;
                haveDistance = true;
                targetRpm = rpmForDistance(smoothedDistance);
            }
        }
    }

    private double aimTurnPower(double error) {
        if (Math.abs(error) < AIM_TOLERANCE_DEG) return 0.0;
        double power = AIM_DIRECTION * AIM_KP * error;
        if (Math.abs(power) < AIM_MIN_POWER) power = Math.copySign(AIM_MIN_POWER, power);
        return Range.clip(power, -AIM_MAX_POWER, AIM_MAX_POWER);
    }

    // ----------------------------------------------------------------------------------
    // 4. SHOOTER
    // ----------------------------------------------------------------------------------
    /** Linear interpolation through SHOOTER_TABLE, clamped at both ends. */
    private double rpmForDistance(double meters) {
        if (meters <= SHOOTER_TABLE[0][0]) return SHOOTER_TABLE[0][1];
        int last = SHOOTER_TABLE.length - 1;
        if (meters >= SHOOTER_TABLE[last][0]) return SHOOTER_TABLE[last][1];
        for (int i = 0; i < last; i++) {
            double d0 = SHOOTER_TABLE[i][0], d1 = SHOOTER_TABLE[i + 1][0];
            if (meters <= d1) {
                double t = (meters - d0) / (d1 - d0);
                return SHOOTER_TABLE[i][1] + t * (SHOOTER_TABLE[i + 1][1] - SHOOTER_TABLE[i][1]);
            }
        }
        return DEFAULT_SHOOTER_RPM;
    }

    private void updateShooter() {
        setShooterRpm(shooterEnabled ? targetRpm : 0.0);
    }

    private void setShooterRpm(double rpm) {
        double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        innerShooter.setVelocity(ticksPerSecond);
        outerShooter.setVelocity(ticksPerSecond);
    }

    private double rpmOf(DcMotorEx motor) {
        return Math.abs(motor.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private boolean shooterAtSpeed() {
        if (!shooterEnabled) return false;
        return Math.abs(rpmOf(innerShooter) - targetRpm) < SHOOTER_RPM_TOLERANCE
                && Math.abs(rpmOf(outerShooter) - targetRpm) < SHOOTER_RPM_TOLERANCE;
    }

    // ----------------------------------------------------------------------------------
    // 5. INTAKE
    // LT = intake both stages, RT = feed the shooter (only once it is at speed),
    // B = reverse. Reverse wins, then shoot, then intake.
    // ----------------------------------------------------------------------------------
    private void updateIntake(boolean shooterAtSpeed) {
        boolean intake = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean shoot = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        double inner = 0.0;
        double outer = 0.0;

        if (gamepad1.b) {
            inner = -INTAKE_POWER;
            outer = -INTAKE_POWER;
        } else {
            if (intake) {
                inner = INTAKE_POWER;
                outer = INTAKE_POWER;
            }
            // Only feed the flywheels once they are up to speed, so shots are consistent.
            if (shoot && shooterAtSpeed) {
                inner = FEED_POWER;
            } else if (shoot) {
                inner = 0.0;   // Wait for the flywheels; don't push a piece into slow wheels.
            }
        }

        innerIntake.setPower(inner);
        outerIntake.setPower(outer);
    }

    // ----------------------------------------------------------------------------------
    // 6. BUTTON HANDLING (act on the press, not while held)
    // ----------------------------------------------------------------------------------
    private void handleToggles() {
        if (gamepad1.right_bumper && !lastRightBumper) aimEnabled = !aimEnabled;
        if (gamepad1.left_bumper && !lastLeftBumper) shooterEnabled = !shooterEnabled;
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
    }

    private void handlePipelineButtons() {
        if (gamepad1.dpad_left && pipeline != APRILTAG_PIPELINE) {
            pipeline = APRILTAG_PIPELINE;
            limelight.pipelineSwitch(pipeline);
            haveHive = false;
            haveDistance = false;
        }
        if (gamepad1.dpad_right && pipeline != POLLEN_PIPELINE) {
            pipeline = POLLEN_PIPELINE;
            limelight.pipelineSwitch(pipeline);
            haveHive = false;
            haveDistance = false;
        }
    }

    // ----------------------------------------------------------------------------------
    // 7. HARDWARE SETUP
    // ----------------------------------------------------------------------------------
    private void initMechanisms() {
        innerShooter = hardwareMap.get(DcMotorEx.class, INNER_SHOOTER_NAME);
        outerShooter = hardwareMap.get(DcMotorEx.class, OUTER_SHOOTER_NAME);
        innerShooter.setDirection(INNER_SHOOTER_DIR);
        outerShooter.setDirection(OUTER_SHOOTER_DIR);
        for (DcMotorEx m : new DcMotorEx[]{innerShooter, outerShooter}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // Needed for setVelocity()
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        innerIntake = hardwareMap.get(DcMotor.class, INNER_INTAKE_NAME);
        outerIntake = hardwareMap.get(DcMotor.class, OUTER_INTAKE_NAME);
        innerIntake.setDirection(INNER_INTAKE_DIR);
        outerIntake.setDirection(OUTER_INTAKE_DIR);
        for (DcMotor m : new DcMotor[]{innerIntake, outerIntake}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // Drive motor names/directions match pedro/Constants.java. Keep them in sync.
    private void initDrive() {
        leftFront = hardwareMap.get(DcMotor.class, "motor0");
        rightFront = hardwareMap.get(DcMotor.class, "motor1");
        leftRear = hardwareMap.get(DcMotor.class, "motor2");
        rightRear = hardwareMap.get(DcMotor.class, "motor3");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void drive(double forward, double strafe, double turn) {
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        leftFront.setPower((forward + strafe + turn) / max);
        leftRear.setPower((forward - strafe + turn) / max);
        rightFront.setPower((forward - strafe - turn) / max);
        rightRear.setPower((forward + strafe - turn) / max);
    }
}
