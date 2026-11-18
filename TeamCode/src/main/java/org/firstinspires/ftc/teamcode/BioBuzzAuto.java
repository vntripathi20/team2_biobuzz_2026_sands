package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * ======================================================================================
 *                         BIOBUZZ AUTONOMOUS (Pedro Pathing)
 * ======================================================================================
 * Sequence:  drive to the pollen pickup (intake on) -> collect -> drive to the hive
 *            shooting spot (flywheels spin up on the way) -> aim the turret at the hive ->
 *            wait for flywheels -> feed and shoot -> (optional) return to the start.
 *
 * RUN "BioBuzz Auto BLUE" or "BioBuzz Auto RED" (BioBuzzAutoBlue / BioBuzzAutoRed). This
 * class holds all the logic; those two just pick the alliance.
 *
 * INIT CONTROLS (gamepad 1):
 *   A = toggle the pickup run     Y = toggle returning to the start spot at the end
 *
 * START POSITION: against your alliance wall, centered on the field's middle seam
 * (Y = 72), facing into the field. RED is on the x = 0 side, BLUE on the x = 144 side.
 *
 * POSES come from the team's other repo (BioBuzz field measurements). BLUE is RED rotated
 * 180 degrees around the field center (72, 72), where the hive is.
 *
 * THE TURRET is aimed from the robot's position, not the camera: it points at (72, 72)
 * using the Pedro pose. It assumes the turret's zero is straight ahead and the hive is at
 * the field center.
 *
 * Needs pedro/Constants.java tuned (it is, copied from the team repo) and the hardware
 * names to match the Robot Configuration (Pinpoint named "odo").
 * ======================================================================================
 */
public abstract class BioBuzzAuto extends OpMode {

    /** Called by BioBuzzAutoBlue / BioBuzzAutoRed, which are the two OpModes you run. */
    protected BioBuzzAuto(boolean redAlliance) {
        this.redAlliance = redAlliance;
    }

    // ----------------------------------------------------------------------------------
    // 1. FIELD POSES (inches, degrees). RED values; BLUE is mirrored by rotating 180 deg
    //    about (72, 72).
    // ----------------------------------------------------------------------------------
    private static final double[] RED_START = {9.0, 72.0, 0.0};
    private static final double[] RED_PICKUP = {24.0, 48.0, 0.0};
    private static final double[] RED_HIVE_SHOT = {59.25, 18.0, 90.0};
    private static final double FIELD_CENTER_X = 72.0;
    private static final double FIELD_CENTER_Y = 72.0;

    // ----------------------------------------------------------------------------------
    // 2. SETTINGS - tune on the field.
    // ----------------------------------------------------------------------------------
    private static final double COLLECT_SECONDS = 1.0;      // pause at the pickup with intake on
    private static final double INTAKE_POWER = 0.8;
    private static final double AUTO_SHOOTER_RPM = 3000.0;  // TODO: best RPM from the shot spot
    private static final double SHOOTER_TICKS_PER_REV = 28.0;   // TODO: match TeleOp
    private static final double SHOOTER_RPM_TOLERANCE = 100.0;
    private static final double SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double FEED_SECONDS = 3.0;
    private static final double FEED_POWER = 1.0;
    private static final boolean FEED_OUTER_TOO = true;
    private static final double PATH_START_GRACE_SECONDS = 0.5;   // wait for the follower to start
    private static final double PATH_TIMEOUT_SECONDS = 12.0;   // abort a path that never ends
    private static final double AUTO_TOTAL_SECONDS = 29.0;     // stop everything before 30 s
    private static final boolean RETURN_TO_START_DEFAULT = false;  // TODO: park spot unknown

    // --- Turret (Axon Max servos, positional) - keep in sync with BioBuzzTeleOp ---
    private static final String TURRET_SERVO_RIGHT_NAME = "servo5B";
    private static final String TURRET_SERVO_LEFT_NAME = "servo0B";
    private static final boolean USE_LEFT_TURRET_SERVO = false;   // not plugged in yet
    private static final boolean LEFT_SERVO_OPPOSITE = true;
    private static final boolean TURRET_REVERSED = false;
    private static final double TURRET_GEAR_RATIO = 1.0;          // TODO
    private static final double SERVO_RANGE_DEG = 355.0;          // TODO
    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG = 90.0;
    // Adds to the computed angle if the shooter isn't exactly on the turret's zero line.
    private static final double TURRET_AIM_OFFSET_DEG = 0.0;      // TODO
    private static final boolean AUTO_AIM_TURRET = true;

    // --- Hardware names and directions (match BioBuzzTeleOp) ---
    private static final String INNER_SHOOTER_NAME = "motor0B";
    private static final String OUTER_SHOOTER_NAME = "motor1B";
    private static final String INNER_INTAKE_NAME = "motor2B";
    private static final String OUTER_INTAKE_NAME = "motor3B";
    private static final DcMotorSimple.Direction INNER_SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction OUTER_SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    // The outer shooter motor is loose right now, so it is switched OFF. Set to true to use it.
    private static final boolean USE_OUTER_SHOOTER = false;
    private static final DcMotorSimple.Direction INNER_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction OUTER_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;

    // ----------------------------------------------------------------------------------
    // 3. STATE
    // ----------------------------------------------------------------------------------
    private static final int IDLE = 0;
    private static final int DRIVE_TO_PICKUP = 1;
    private static final int COLLECT = 2;
    private static final int DRIVE_TO_HIVE = 3;
    private static final int SPIN_UP = 4;
    private static final int FEED = 5;
    private static final int RETURN_TO_START = 6;
    private static final int COMPLETE = 7;
    private static final int ABORTED = 8;

    private final PoseFactory poses = PoseFactory.degrees();
    private Follower follower;
    private DcMotorEx innerShooter, outerShooter;
    private DcMotor innerIntake, outerIntake;
    private Servo turretRight, turretLeft;
    private GoBildaPinpointDriver pinpointCheck;   // only used to show its status on screen

    private boolean pathActive = false;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private final boolean redAlliance;
    private boolean runPickup = true;
    private boolean returnToStart = RETURN_TO_START_DEFAULT;
    private boolean prevA, prevY;
    private int stage = IDLE;
    private String status = "Ready";

    // ----------------------------------------------------------------------------------
    // 4. OPMODE
    // ----------------------------------------------------------------------------------
    @Override
    public void init() {
        Scheduler.reset();
        initMechanisms();
        follower = Constants.create(hardwareMap);
        pinpointCheck = hardwareMap.tryGet(GoBildaPinpointDriver.class, "odo");
        follower.setPose(startPose());
        follower.update();
        setTurretAngle(0.0);   // the turret must physically be straight ahead at INIT
    }

    @Override
    public void init_loop() {
        if (gamepad1.a && !prevA) runPickup = !runPickup;
        if (gamepad1.y && !prevY) returnToStart = !returnToStart;
        prevA = gamepad1.a;
        prevY = gamepad1.y;

        follower.setPose(startPose());   // keep the pose tied to the chosen alliance
        follower.update();
        if (pinpointCheck != null) {
            pinpointCheck.update();
            telemetry.addData("Pinpoint status", pinpointCheck.getDeviceStatus());
        } else {
            telemetry.addData("Pinpoint status", "NOT FOUND under the name odo");
        }
        telemetry.addData("Pose now", "x %.1f, y %.1f, heading %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Start", poseText(startPose()));
        telemetry.addData("Pickup", poseText(pickupPose()));
        telemetry.addData("Hive shot", poseText(hivePose()));
        telemetry.addData("Route", "pickup %s, return to start %s",
                runPickup ? "on" : "skip", returnToStart ? "on" : "off");
        telemetry.addData("Turret servo", turretRight != null ? "found" : "NOT FOUND (servo5B)");
        telemetry.addLine("A = pickup on/off | Y = return to start on/off");
        telemetry.addLine("Against your wall, centered on Y=72, facing in. Turret straight ahead.");
        telemetry.update();
    }

    @Override
    public void start() {
        Scheduler.reset();
        follower.setPose(startPose());
        follower.update();
        autoTimer.reset();
        if (runPickup) {
            stage = DRIVE_TO_PICKUP;
            status = "Driving to pickup";
            schedulePath(pickupPose());
        } else {
            beginHiveTransit();
        }
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();
        if (!finite(pose.x()) || !finite(pose.y()) || !finite(pose.heading())) {
            abort("LOCALIZER INVALID - stopped (check the Pinpoint)");
        } else {
            Scheduler.execute();
        }
        // A path counts as running for a short grace period after it starts (the follower needs a
        // loop or two to report busy), then for as long as the follower says it is busy.
        boolean pathRunning = pathActive
                && (pathTimer.seconds() < PATH_START_GRACE_SECONDS || follower.isBusy());

        if (isPathStage() && pathRunning && pathTimer.seconds() >= PATH_TIMEOUT_SECONDS) {
            abort("PATH TIMEOUT - stopped");
            pathRunning = false;
        }
        if (autoTimer.seconds() >= AUTO_TOTAL_SECONDS && stage != COMPLETE && stage != ABORTED) {
            abort("AUTO TIMEOUT - stopped before the end of auto");
            pathRunning = false;
        }

        updateState(pathRunning);
        applyOutputs();

        telemetry.addData("Alliance / status", "%s / %s", redAlliance ? "RED" : "BLUE", status);
        telemetry.addData("Stage", stageName());
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Elapsed", "%.1f s", autoTimer.seconds());
        telemetry.addData("Inner / Outer RPM", "%.0f / %.0f", rpmOf(innerShooter), rpmOf(outerShooter));
        telemetry.addData("Turret aim", "%.1f deg", turretAimDeg(pose));
        telemetry.addData("Path running / follower busy", "%s / %s", pathRunning, follower.isBusy());
        telemetry.addData("Distance to path end", "%.1f in", follower.distanceToEndpoint());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        stopMechanisms();
        if (follower != null) follower.stop();
    }

    // ----------------------------------------------------------------------------------
    // 5. STATE MACHINE
    // ----------------------------------------------------------------------------------
    private void updateState(boolean pathRunning) {
        if (stage == DRIVE_TO_PICKUP && !pathRunning) {
            pathActive = false;
            stage = COLLECT;
            phaseTimer.reset();
            status = "Collecting pollen";
        } else if (stage == COLLECT && phaseTimer.seconds() >= COLLECT_SECONDS) {
            beginHiveTransit();
        } else if (stage == DRIVE_TO_HIVE && !pathRunning) {
            pathActive = false;
            stage = SPIN_UP;
            phaseTimer.reset();
            status = "At the hive: waiting for flywheels";
        } else if (stage == SPIN_UP
                && (shooterAtSpeed() || phaseTimer.seconds() >= SPINUP_TIMEOUT_SECONDS)) {
            stage = FEED;
            phaseTimer.reset();
            status = "Feeding and shooting";
        } else if (stage == FEED && phaseTimer.seconds() >= FEED_SECONDS) {
            if (returnToStart) {
                stage = RETURN_TO_START;
                status = "Returning to start";
                schedulePath(startPose());
            } else {
                complete("Score sequence complete");
            }
        } else if (stage == RETURN_TO_START && !pathRunning) {
            pathActive = false;
            complete("Score and return complete");
        }
    }

    private void beginHiveTransit() {
        stage = DRIVE_TO_HIVE;
        status = "Driving to the hive spot; flywheels spinning";
        schedulePath(hivePose());
    }

    /** Sets what the intake, shooter and turret should be doing in the current stage. */
    private void applyOutputs() {
        double inner = 0.0;
        double outer = 0.0;
        double rpm = 0.0;

        if (stage == DRIVE_TO_PICKUP || stage == COLLECT) {
            inner = INTAKE_POWER;
            outer = INTAKE_POWER;
        } else if (stage == DRIVE_TO_HIVE || stage == SPIN_UP) {
            rpm = AUTO_SHOOTER_RPM;
        } else if (stage == FEED) {
            rpm = AUTO_SHOOTER_RPM;
            inner = FEED_POWER;
            outer = FEED_OUTER_TOO ? INTAKE_POWER : 0.0;
        }

        setShooterRpm(rpm);
        innerIntake.setPower(inner);
        outerIntake.setPower(outer);
        if (AUTO_AIM_TURRET && stage != COMPLETE && stage != ABORTED) {
            setTurretAngle(turretAimDeg(follower.pose()));
        }
    }

    // ----------------------------------------------------------------------------------
    // 6. PATHS
    // ----------------------------------------------------------------------------------
    private void schedulePath(Pose to) {
        // Start from the measured pose; the last path may have finished a little short.
        Pose from = follower.pose();
        Path path = line(from, to).linear(from, to);
        follower.follow(path);
        pathActive = true;
        pathTimer.reset();
    }

    private boolean isPathStage() {
        return stage == DRIVE_TO_PICKUP || stage == DRIVE_TO_HIVE || stage == RETURN_TO_START;
    }

    private Pose startPose() { return fieldPose(RED_START); }
    private Pose pickupPose() { return fieldPose(RED_PICKUP); }
    private Pose hivePose() { return fieldPose(RED_HIVE_SHOT); }

    /** RED as given; BLUE is the same point rotated 180 degrees about the field center. */
    private Pose fieldPose(double[] red) {
        if (!redAlliance) {
            return poses.of(2 * FIELD_CENTER_X - red[0], 2 * FIELD_CENTER_Y - red[1], red[2] + 180.0);
        }
        return poses.of(red[0], red[1], red[2]);
    }

    // ----------------------------------------------------------------------------------
    // 7. TURRET AIM FROM THE ROBOT'S POSE
    // ----------------------------------------------------------------------------------
    /** Turret angle (degrees clockwise from straight ahead) that points at the hive. */
    private double turretAimDeg(Pose pose) {
        double toHiveDeg = Math.toDegrees(Math.atan2(FIELD_CENTER_Y - pose.y(), FIELD_CENTER_X - pose.x()));
        double headingDeg = Math.toDegrees(pose.heading());
        double clockwise = wrapDeg(headingDeg - toHiveDeg) + TURRET_AIM_OFFSET_DEG;
        return Range.clip(clockwise, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    private void setTurretAngle(double turretDeg) {
        double clipped = Range.clip(turretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
        double sign = TURRET_REVERSED ? -1.0 : 1.0;
        double pos = Range.clip(0.5 + sign * clipped * TURRET_GEAR_RATIO / SERVO_RANGE_DEG, 0.0, 1.0);
        if (turretRight != null) turretRight.setPosition(pos);
        if (turretLeft != null) turretLeft.setPosition(LEFT_SERVO_OPPOSITE ? 1.0 - pos : pos);
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    // ----------------------------------------------------------------------------------
    // 8. MECHANISMS
    // ----------------------------------------------------------------------------------
    private void initMechanisms() {
        innerShooter = hardwareMap.get(DcMotorEx.class, INNER_SHOOTER_NAME);
        outerShooter = hardwareMap.get(DcMotorEx.class, OUTER_SHOOTER_NAME);
        innerShooter.setDirection(INNER_SHOOTER_DIR);
        outerShooter.setDirection(OUTER_SHOOTER_DIR);
        for (DcMotorEx m : new DcMotorEx[]{innerShooter, outerShooter}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // needed for setVelocity()
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

        turretRight = hardwareMap.tryGet(Servo.class, TURRET_SERVO_RIGHT_NAME);
        turretLeft = USE_LEFT_TURRET_SERVO
                ? hardwareMap.tryGet(Servo.class, TURRET_SERVO_LEFT_NAME) : null;
    }

    private void setShooterRpm(double rpm) {
        double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        innerShooter.setVelocity(ticksPerSecond);
        if (USE_OUTER_SHOOTER) outerShooter.setVelocity(ticksPerSecond);
    }

    private double rpmOf(DcMotorEx motor) {
        return Math.abs(motor.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private boolean shooterAtSpeed() {
        return Math.abs(rpmOf(innerShooter) - AUTO_SHOOTER_RPM) < SHOOTER_RPM_TOLERANCE
                && (!USE_OUTER_SHOOTER
                || Math.abs(rpmOf(outerShooter) - AUTO_SHOOTER_RPM) < SHOOTER_RPM_TOLERANCE);
    }

    private void stopMechanisms() {
        if (innerShooter != null) setShooterRpm(0.0);
        if (innerIntake != null) innerIntake.setPower(0.0);
        if (outerIntake != null) outerIntake.setPower(0.0);
    }

    // ----------------------------------------------------------------------------------
    // 9. FINISHING
    // ----------------------------------------------------------------------------------
    private void abort(String reason) {
        if (stage == ABORTED) return;
        Scheduler.reset();
        pathActive = false;
        follower.stop();
        stage = ABORTED;
        status = reason;
        stopMechanisms();
    }

    private void complete(String message) {
        stage = COMPLETE;
        status = message;
        stopMechanisms();
        follower.stop();
    }

    private String stageName() {
        switch (stage) {
            case DRIVE_TO_PICKUP: return "Drive to pickup";
            case COLLECT: return "Collect";
            case DRIVE_TO_HIVE: return "Drive to hive";
            case SPIN_UP: return "Wait for flywheels";
            case FEED: return "Feed and shoot";
            case RETURN_TO_START: return "Return";
            case COMPLETE: return "Complete";
            case ABORTED: return "Aborted";
            default: return "Idle";
        }
    }

    private String poseText(Pose pose) {
        return String.format("(%.1f, %.1f, %.0f deg)", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
