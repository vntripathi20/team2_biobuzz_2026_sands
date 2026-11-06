package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;

/**
 * ======================================================================================
 *                       BIOBUZZ TELEOP (Sense and Sound Robotics)
 * ======================================================================================
 * Mecanum drive + intake + a shooter TURRET that keeps itself pointed at the HIVE.
 *
 *   AIMING:    The Limelight (pipeline 0 = AprilTags) finds the hive tags of your alliance.
 *              Only the TURRET moves to point the shooter at the hive. The robot body
 *              NEVER turns by itself: you always steer with the right stick.
 *              If the tag leaves view, the turret keeps pointing at the direction the
 *              hive was last seen, and the gyro (IMU) corrects for the robot turning
 *              (see TURRET_USE_GYRO_HOLD).
 *   SHOOTER:   The flywheel RPM is picked from the distance to the hive tag using the
 *              SHOOTER_TABLE below. The feed only runs once the turret is aimed and the
 *              flywheels are up to speed.
 *   INTAKE:    Two stages. Outer = motor1B (picks pollen off the floor), inner = motor0B
 *              (moves it toward the shooter).
 *
 * RUN "BioBuzz TeleOp BLUE" or "BioBuzz TeleOp RED" (BioBuzzTeleOpBlue / BioBuzzTeleOpRed).
 * This class holds all the logic; those two just pick the alliance.
 *
 * CONTROLS (gamepad 1):
 *   Left stick        drive / strafe
 *   Right stick X     turn the robot (always yours)
 *   Right bumper      toggle turret auto-aim ON / OFF   (starts ON; OFF = turret holds)
 *   Left bumper       toggle shooter spin-up ON / OFF (starts OFF)
 *   Right trigger     AUTO-SHOOT: hold it. Spins up the shooter, then feeds once the
 *                     turret is aimed at the hive AND the flywheels are at speed
 *   A                 MANUAL shoot: hold it. Feeds once the flywheels are at speed, with
 *                     no aim check (backup if the camera or turret isn't working)
 *   Left trigger      INTAKE (outer + inner)
 *   B                 reverse intake (spit out jams)
 *   D-pad left        AprilTag pipeline (aiming)
 *   D-pad right       Pollen pipeline. Disabled until POLLEN_ENABLED is set to true.
 *
 * HIVE TAGS:  BLUE = IDs 38-45,  RED = IDs 30-37 (both sides of each hive).
 *
 * BEFORE THIS WILL AIM ON THE ROBOT:
 *   - START WITH THE TURRET POINTING STRAIGHT AHEAD. The Axon servos have no feedback
 *     here, so the code assumes the turret starts straight ahead at init.
 *   - Fill in every "TODO" in the TURRET section: TURRET_GEAR_RATIO, SERVO_RANGE_DEG,
 *     the limits, the direction, and where the camera is mounted.
 *   - Shooter motors (motor2B, motor3B) need their encoders plugged in, because the
 *     flywheel runs on velocity control.
 * ======================================================================================
 */
public abstract class BioBuzzTeleOp extends LinearOpMode {

    /** Called by BioBuzzTeleOpBlue / BioBuzzTeleOpRed, which are the two OpModes you run. */
    protected BioBuzzTeleOp(boolean redAlliance) {
        this.redAlliance = redAlliance;
    }

    // ----------------------------------------------------------------------------------
    // 1. SETTINGS - tune these on your robot.
    // ----------------------------------------------------------------------------------

    // --- Limelight ---
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String LIMELIGHT_HOST = "172.29.0.1";
    private static final int LIMELIGHT_STREAM_PORT = 5800;
    private static final int APRILTAG_PIPELINE = 0;
    private static final int POLLEN_PIPELINE = 1;      // TODO: set to your pollen model's pipeline
    // The camera is mounted to see the hive tags, so it can't see pollen on the floor.
    // Leave this false so D-pad right can't switch away from AprilTags and pause aiming.
    // Set it to true once pollen detection has its own camera/pipeline.
    private static final boolean POLLEN_ENABLED = false;

    // --- Hive tag IDs (both sides of each hive) ---
    private static final int BLUE_HIVE_MIN_ID = 38;
    private static final int BLUE_HIVE_MAX_ID = 45;
    private static final int RED_HIVE_MIN_ID = 30;
    private static final int RED_HIVE_MAX_ID = 37;

    // --- Camera / tag tracking ---
    // The tx you see when the shooter is exactly on target, for a camera mounted ON THE
    // TURRET (see CAMERA_ON_TURRET). It isn't zero if the camera isn't lined up with the
    // shooter. Aim the shooter at the hive by hand, read tx, and enter it. Leave it 0 for a
    // camera mounted on the robot body and use CAMERA_YAW_OFFSET_DEG instead.
    private static final double AIM_TARGET_TX_DEG = 0.0;   // TODO
    private static final double TX_SMOOTHING = 0.5;
    // For a camera on the turret: "aimed" means the tag is within this many degrees of the
    // target tx.
    private static final double AIM_TOLERANCE_DEG = 1.5;
    // We only call the shooter "aimed and ready" if the tag was seen this recently.
    private static final double READY_MAX_AGE_SECONDS = 0.5;

    // --- Turret (rotates the shooter side to side) ---
    // Angles are measured in degrees, clockwise (to the right) from straight ahead.
    // The turret must point straight ahead when you press INIT: that is angle zero.
    // The turret is turned by two Axon Max servos in POSITIONAL mode (no feedback wire is
    // needed). The right one is plugged in first; the left is optional and is skipped if it
    // isn't in the Robot Configuration yet.
    private static final String TURRET_SERVO_RIGHT_NAME = "servo5B";
    private static final String TURRET_SERVO_LEFT_NAME = "servo0B";
    // The left servo is not plugged in yet, so it stays OFF. Set this to true once it's wired.
    private static final boolean USE_LEFT_TURRET_SERVO = false;
    // TODO: flip if a positive angle moves the shooter to the LEFT. This flips both servos.
    private static final boolean TURRET_REVERSED = false;
    // The two servos sit on opposite sides of the ring, so one usually needs reversing to
    // turn the same way. TODO: flip this if the two fight each other once both are wired.
    private static final boolean LEFT_SERVO_OPPOSITE = true;
    // TODO: gear ratio = turret ring teeth / servo gear teeth. The servo turns this many
    // degrees for every 1 degree of turret. 1.0 is a placeholder.
    private static final double TURRET_GEAR_RATIO = 1.0;
    // TODO: the servo's full travel in degrees for setPosition 0 -> 1. Check Axon Studio.
    private static final double SERVO_RANGE_DEG = 355.0;
    // The servos have no feedback, so we assume the turret moves at this speed to estimate
    // its angle. TODO: time a real sweep and update. Lower = more cautious about "aimed".
    private static final double TURRET_SPEED_DEG_PER_SEC = 180.0;
    // TODO: how far the turret may turn each way, so it never hits a stop or winds wires.
    // Keep this inside what the servo's travel allows after the gear ratio.
    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG = 90.0;
    // "Aimed" means the turret is within this many degrees of where it should point.
    private static final double TURRET_AIM_TOLERANCE_DEG = 2.0;
    // TODO: true if the Limelight is bolted to the turret (it turns with the shooter).
    // False if it's fixed to the robot body.
    private static final boolean CAMERA_ON_TURRET = false;
    // Which way the camera faces compared with the turret's zero ("straight ahead"):
    // 0 = the same way, 180 = backwards. Positive = camera points to the right.
    private static final double CAMERA_YAW_OFFSET_DEG = 0.0;   // TODO
    // True: when the tag is out of view the turret keeps pointing at the hive's last known
    // direction, correcting with the gyro as the robot turns. False: it just holds the last
    // turret angle.
    private static final boolean TURRET_USE_GYRO_HOLD = true;

    // --- IMU (gyro) used to hold the aim when the hive tag is out of view ---
    private static final String IMU_NAME = "imu";
    // TODO: how the Control Hub is mounted on the robot. Look at the hub's logo and USB
    // port and set these to the directions they face. Wrong values make the gyro hold
    // aim the wrong way. Check with the telemetry "Gyro yaw": turning the robot LEFT
    // (counterclockwise) must make it go UP.
    private static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // --- Shooter ---
    private static final String INNER_SHOOTER_NAME = "motor2B";
    private static final String OUTER_SHOOTER_NAME = "motor3B";
    // TODO: flywheels usually spin opposite ways. Flip one if a wheel spins backwards.
    private static final DcMotorSimple.Direction INNER_SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction OUTER_SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    // If a shooter motor is dead or unplugged, set its flag to false and the other one runs
    // alone (and "at speed" only checks the working one).
    private static final boolean USE_INNER_SHOOTER = true;
    private static final boolean USE_OUTER_SHOOTER = true;
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
    private Servo turretRight, turretLeft;   // right is required; left is optional
    private final ElapsedTime turretClock = new ElapsedTime();
    private double turretAngleEstimate = 0.0;   // where we believe the turret points (deg)
    private IMU imu;

    private final LimelightTeleOp.LimelightStream stream =
            new LimelightTeleOp.LimelightStream(LIMELIGHT_HOST, LIMELIGHT_STREAM_PORT);

    private final boolean redAlliance;
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
    // Direction of the hive in the gyro's frame (degrees), remembered from the last time
    // the tag was seen. Used to keep the turret on the hive when the tag is out of view.
    private boolean haveHiveYaw = false;
    private double hiveYawDeg = 0.0;
    // Where the turret is being told to point (degrees clockwise from straight ahead).
    private double turretTargetDeg = 0.0;
    private boolean haveDistance = false;
    private double smoothedDistance = 0.0;
    private double targetRpm = DEFAULT_SHOOTER_RPM;

    @Override
    public void runOpMode() {
        initDrive();
        initMechanisms();
        initTurret();
        initImu();

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        stream.start();
        CameraStreamServer.getInstance().setSource(stream);

        // INIT: the alliance is fixed by which OpMode you picked, so we only aim at OUR
        // hive tags. Check this line shows the alliance you expect, then press PLAY.
        while (opModeInInit()) {
            telemetry.addData("Alliance", redAlliance ? "RED (tags 30-37)" : "BLUE (tags 38-45)");
            telemetry.addData("Turret", turretFound()
                    ? "found - make sure it points straight ahead"
                    : "NOT FOUND (check servo5B in the Robot Configuration)");
            telemetry.addLine("Press PLAY when ready");
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

            double yaw = imuYawDeg();                // degrees, counterclockwise = increasing
            double turretAngle = turretAngleDeg();   // degrees, clockwise from straight ahead

            // ------------------------------------------------------------------------------
            // HIVE TRACKING
            // ------------------------------------------------------------------------------
            if (hive != null) {
                updateHiveTracking(hive);
                // Where the hive is, relative to the robot's front, clockwise positive. A
                // camera on the turret reports the tag relative to the turret, so add the
                // turret angle to get back to the robot's frame.
                double bearing = (CAMERA_ON_TURRET ? turretAngle : 0.0)
                        + CAMERA_YAW_OFFSET_DEG + lastError;
                // Remember it in the gyro's frame so it survives the robot turning.
                // (yaw is counterclockwise, bearing is clockwise, hence the minus.)
                hiveYawDeg = wrapDeg(yaw - bearing);
                haveHiveYaw = true;
            }

            // ------------------------------------------------------------------------------
            // TURRET: the only thing that aims. The robot body is never turned from here.
            // ------------------------------------------------------------------------------
            String aimMode = "none";
            if (aimEnabled && pipeline == APRILTAG_PIPELINE && haveHiveYaw) {
                if (hive != null || TURRET_USE_GYRO_HOLD) {
                    // Hive direction in the robot's frame, from the gyro (or the live tag).
                    turretTargetDeg = Range.clip(wrapDeg(yaw - hiveYawDeg),
                            TURRET_MIN_DEG, TURRET_MAX_DEG);
                }
                aimMode = hive != null ? "TRACKING" : "HOLDING";
            }
            driveTurret(turretTargetDeg);

            // ------------------------------------------------------------------------------
            // DRIVING - always the driver's sticks
            // ------------------------------------------------------------------------------
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);

            // ------------------------------------------------------------------------------
            // SHOOTER + INTAKE
            // ------------------------------------------------------------------------------
            // RT = AUTO-SHOOT: spins the shooter up and feeds only once the turret is aimed
            // at the hive AND the flywheels are at speed. A = MANUAL shoot: feeds once the
            // flywheels are at speed, no aim check (a backup if the camera isn't working).
            boolean autoShootHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean manualShootHeld = gamepad1.a;
            boolean spinning = shooterEnabled || autoShootHeld || manualShootHeld;

            updateShooter(spinning);
            boolean atSpeed = shooterAtSpeed(spinning);

            double turretError = Math.abs(turretTargetDeg - turretAngle);
            boolean aimed = aimEnabled && turretFound() && haveHive
                    && sinceSeen.seconds() < READY_MAX_AGE_SECONDS
                    && turretError < TURRET_AIM_TOLERANCE_DEG
                    && (!CAMERA_ON_TURRET || Math.abs(lastError) < AIM_TOLERANCE_DEG);
            boolean readyToShoot = spinning && atSpeed && aimed;

            // Rumble once each time we become ready, so the driver can feel it.
            if (readyToShoot && !wasReady) gamepad1.rumble(150);
            wasReady = readyToShoot;

            boolean feedNow = (autoShootHeld && atSpeed && aimed) || (manualShootHeld && atSpeed);
            updateIntake(feedNow);

            // ------------------------------------------------------------------------------
            // TELEMETRY
            // ------------------------------------------------------------------------------
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("Pipeline", pipeline == APRILTAG_PIPELINE ? "AprilTag" : "Pollen");
            telemetry.addData("Turret aim", !aimEnabled ? "OFF (holding)"
                    : aimMode.equals("TRACKING") ? "TRACKING (tag in view)"
                    : aimMode.equals("HOLDING") ? "HOLDING (tag out of view)"
                    : "no hive seen yet");
            if (!turretFound()) {
                telemetry.addLine("TURRET NOT FOUND - auto-shoot can't fire, use A to shoot");
            } else {
                telemetry.addData("Turret angle / target", "%.1f / %.1f deg", turretAngle, turretTargetDeg);
            }
            telemetry.addData("Hive tx error", haveHive ? String.format("%.1f deg", lastError) : "-");
            telemetry.addData("Gyro yaw (left turn = up)", "%.1f deg", yaw);
            telemetry.addData("Hive distance", haveDistance ? String.format("%.2f m", smoothedDistance) : "-");
            telemetry.addData("Shooter", spinning ? "SPINNING" : "off");
            telemetry.addData("Shoot", feedNow ? "FIRING"
                    : autoShootHeld ? (!atSpeed ? "auto: waiting for flywheels"
                                                : !aimed ? "auto: waiting for aim" : "auto")
                    : manualShootHeld ? "manual: waiting for flywheels"
                    : "-");
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

    // ----------------------------------------------------------------------------------
    // Gyro helpers (used to keep the turret on the hive when the tag is out of view)
    // ----------------------------------------------------------------------------------
    private void initImu() {
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION)));
        imu.resetYaw();
    }

    private double imuYawDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Wraps an angle in degrees into the range (-180, 180]. */
    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    // ----------------------------------------------------------------------------------
    // 4. TURRET
    // ----------------------------------------------------------------------------------
    private void initTurret() {
        // tryGet returns null instead of crashing, so a missing servo only disables aiming.
        turretRight = hardwareMap.tryGet(Servo.class, TURRET_SERVO_RIGHT_NAME);
        turretLeft = USE_LEFT_TURRET_SERVO
                ? hardwareMap.tryGet(Servo.class, TURRET_SERVO_LEFT_NAME) : null;
        turretAngleEstimate = 0.0;   // we assume it starts straight ahead
        turretClock.reset();
        setServoAngle(0.0);
    }

    /** Which turret servo(s) were found. */
    private boolean turretFound() {
        return turretRight != null || turretLeft != null;
    }

    /** Estimated turret angle in degrees, clockwise from straight ahead (0 if no turret). */
    private double turretAngleDeg() {
        return turretFound() ? turretAngleEstimate : 0.0;
    }

    /** Moves the estimate toward the target at TURRET_SPEED_DEG_PER_SEC and commands the servos. */
    private void driveTurret(double targetDeg) {
        double dt = turretClock.seconds();
        turretClock.reset();
        double maxStep = TURRET_SPEED_DEG_PER_SEC * dt;
        double delta = Range.clip(targetDeg - turretAngleEstimate, -maxStep, maxStep);
        turretAngleEstimate += delta;
        setServoAngle(turretAngleEstimate);
    }

    private void setServoAngle(double turretDeg) {
        double sign = TURRET_REVERSED ? -1.0 : 1.0;
        double servoDeg = sign * turretDeg * TURRET_GEAR_RATIO;
        double pos = Range.clip(0.5 + servoDeg / SERVO_RANGE_DEG, 0.0, 1.0);
        if (turretRight != null) turretRight.setPosition(pos);
        if (turretLeft != null) {
            turretLeft.setPosition(LEFT_SERVO_OPPOSITE ? 1.0 - pos : pos);
        }
    }

    // ----------------------------------------------------------------------------------
    // 5. SHOOTER
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

    private void updateShooter(boolean spinning) {
        setShooterRpm(spinning ? targetRpm : 0.0);
    }

    private void setShooterRpm(double rpm) {
        double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        innerShooter.setVelocity(USE_INNER_SHOOTER ? ticksPerSecond : 0.0);
        outerShooter.setVelocity(USE_OUTER_SHOOTER ? ticksPerSecond : 0.0);
    }

    private double rpmOf(DcMotorEx motor) {
        return Math.abs(motor.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private boolean shooterAtSpeed(boolean spinning) {
        if (!spinning) return false;
        boolean innerOk = !USE_INNER_SHOOTER
                || Math.abs(rpmOf(innerShooter) - targetRpm) < SHOOTER_RPM_TOLERANCE;
        boolean outerOk = !USE_OUTER_SHOOTER
                || Math.abs(rpmOf(outerShooter) - targetRpm) < SHOOTER_RPM_TOLERANCE;
        return innerOk && outerOk;
    }

    // ----------------------------------------------------------------------------------
    // 6. INTAKE
    // LT = intake both stages. feedNow = the shoot conditions are met (RT auto-shoot or A
    // manual shoot), so the inner stage pushes a piece into the flywheels.
    // B = reverse. Reverse wins, then shoot, then intake.
    // ----------------------------------------------------------------------------------
    private void updateIntake(boolean feedNow) {
        boolean intake = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean shootHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad1.a;

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
            // Only feed when the shoot conditions are met, so shots are consistent.
            if (feedNow) {
                inner = FEED_POWER;
            } else if (shootHeld) {
                inner = 0.0;   // Waiting for flywheels/aim; don't push a piece in early.
            }
        }

        innerIntake.setPower(inner);
        outerIntake.setPower(outer);
    }

    // ----------------------------------------------------------------------------------
    // 7. BUTTON HANDLING (act on the press, not while held)
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
            haveHiveYaw = false;
            haveDistance = false;
        }
        if (POLLEN_ENABLED && gamepad1.dpad_right && pipeline != POLLEN_PIPELINE) {
            pipeline = POLLEN_PIPELINE;
            limelight.pipelineSwitch(pipeline);
            haveHive = false;
            haveHiveYaw = false;
            haveDistance = false;
        }
    }

    // ----------------------------------------------------------------------------------
    // 8. HARDWARE SETUP
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
