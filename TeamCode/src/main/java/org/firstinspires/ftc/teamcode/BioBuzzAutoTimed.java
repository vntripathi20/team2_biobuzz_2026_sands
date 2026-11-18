package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * ======================================================================================
 *                 BIOBUZZ AUTONOMOUS - TIMED (runs with NO tuning, NO odometry)
 * ======================================================================================
 * Drives by TIME instead of position, so it works today without Pedro Pathing being
 * tuned. It will drift a little, so treat the leg times below as starting points and
 * tune them on the real field.
 *
 *   1. Spin up the shooter and point the turret at the preset angle.
 *   2. Drive to the scoring spot.
 *   3. Wait for the flywheels to reach speed, then feed and shoot the preload.
 *   4. (Optional) Pick up a row of pollen and come back and shoot again.
 *   5. Park.
 *
 * INIT CONTROLS (gamepad 1):
 *   X = BLUE alliance      B = RED alliance      (RED mirrors the sideways moves)
 *
 * BEFORE RUNNING:
 *   - Point the turret straight ahead and put the robot where the comments say, then
 *     press INIT, then PLAY.
 *   - Do the first runs on blocks, then at low speed (DRIVE_POWER) on the field.
 *   - Fill in every "TODO" (leg times, turret angle, RPM).
 *
 * The motor names and directions match BioBuzzTeleOp. Keep them in sync if you change one.
 * ======================================================================================
 */
@Autonomous(name = "BioBuzz Auto (Timed)", group = "Autonomous")
public class BioBuzzAutoTimed extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // 1. SETTINGS - tune these on the field.
    // ----------------------------------------------------------------------------------
    // Drive speed for all legs (0-1). Start low (0.3) and raise it once it behaves.
    private static final double DRIVE_POWER = 0.4;

    // --- Leg times in seconds (TODO: measure on the real robot) ---
    // Leg 1: drive from the start to the scoring spot (forward is +).
    private static final double TO_SCORE_FORWARD_SEC = 1.2;
    // Leg 2: slide sideways toward the hive side after leaving the wall. 0 to skip.
    private static final double TO_SCORE_STRAFE_SEC = 0.0;
    // Park: drive forward (+) for this long after scoring. Negative drives backward.
    private static final double PARK_FORWARD_SEC = 0.8;
    // Park: then slide sideways for this long. 0 to skip.
    private static final double PARK_STRAFE_SEC = 1.0;

    // --- Optional pickup cycle (leave off until the legs above are tuned) ---
    private static final boolean ENABLE_PICKUP_CYCLE = false;
    private static final double PICKUP_STRAFE_SEC = 1.0;     // slide over to the pollen row
    private static final double PICKUP_FORWARD_SEC = 1.2;    // drive through the row, intake on
    private static final double PICKUP_RETURN_SEC = 1.2;     // drive back to the scoring spot
    private static final double PICKUP_RETURN_STRAFE_SEC = 1.0;

    // --- Shooting ---
    // Flywheel RPM used for the whole auto. TODO: set from the best RPM at the scoring spot.
    private static final double AUTO_SHOOTER_RPM = 3000.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0;   // TODO: match TeleOp
    private static final double SHOOTER_RPM_TOLERANCE = 100.0;
    // Max seconds to wait for the flywheels before feeding anyway.
    private static final double SPINUP_TIMEOUT_SEC = 3.0;
    // Seconds to feed pollen into the flywheels for each volley.
    private static final double FEED_SEC = 2.0;
    private static final double FEED_POWER = 1.0;
    // Also run the outer intake while feeding, to push queued pollen up to the shooter.
    private static final boolean FEED_OUTER_TOO = true;
    private static final double INTAKE_POWER = 1.0;

    // --- Turret (Axon Max servos, positional mode) ---
    // Where the turret points while shooting, degrees clockwise from straight ahead.
    // TODO: set this so the shooter points at the hive from the scoring spot. RED may need
    // the opposite sign, so the code flips it for RED when MIRROR_TURRET_FOR_RED is true.
    private static final double TURRET_SHOOT_ANGLE_DEG = 0.0;
    private static final boolean MIRROR_TURRET_FOR_RED = true;
    private static final String TURRET_SERVO_RIGHT_NAME = "servo5B";
    private static final String TURRET_SERVO_LEFT_NAME = "servo0B";
    private static final boolean USE_LEFT_TURRET_SERVO = false;   // not plugged in yet
    private static final boolean LEFT_SERVO_OPPOSITE = true;
    private static final boolean TURRET_REVERSED = false;
    private static final double TURRET_GEAR_RATIO = 1.0;          // TODO: same as TeleOp
    private static final double SERVO_RANGE_DEG = 355.0;          // TODO: same as TeleOp
    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG = 90.0;

    // --- Hardware names and directions (match BioBuzzTeleOp) ---
    private static final String INNER_SHOOTER_NAME = "motor0B";
    private static final String OUTER_SHOOTER_NAME = "motor1B";
    private static final String INNER_INTAKE_NAME = "motor2B";
    private static final String OUTER_INTAKE_NAME = "motor3B";
    private static final DcMotorSimple.Direction INNER_SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction OUTER_SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction INNER_INTAKE_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction OUTER_INTAKE_DIR = DcMotorSimple.Direction.REVERSE;

    // ----------------------------------------------------------------------------------
    // 2. HARDWARE
    // ----------------------------------------------------------------------------------
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotorEx innerShooter, outerShooter;
    private DcMotor innerIntake, outerIntake;
    private Servo turretRight, turretLeft;

    private boolean redAlliance = false;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private String status = "init";

    @Override
    public void runOpMode() {
        initDrive();
        initMechanisms();
        // Turret starts where it physically is, which must be straight ahead.
        setTurretAngle(0.0);

        while (opModeInInit()) {
            if (gamepad1.x) redAlliance = false;
            if (gamepad1.b) redAlliance = true;
            telemetry.addLine("X = BLUE, B = RED");
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("Turret servo", turretRight != null ? "found" : "NOT FOUND (servo5B)");
            telemetry.addLine("Turret straight ahead? Robot on its start spot? Then press PLAY");
            telemetry.update();
        }
        if (isStopRequested()) return;
        autoTimer.reset();

        // Sideways moves flip for RED. Positive strafe = right.
        double side = redAlliance ? -1.0 : 1.0;

        // 1. Spin up and aim while we drive.
        status = "spin up + drive";
        setShooterRpm(AUTO_SHOOTER_RPM);
        setTurretAngle(TURRET_SHOOT_ANGLE_DEG * (redAlliance && MIRROR_TURRET_FOR_RED ? -1.0 : 1.0));

        // 2. Drive to the scoring spot.
        driveFor(DRIVE_POWER, 0.0, TO_SCORE_FORWARD_SEC);
        if (TO_SCORE_STRAFE_SEC > 0) driveFor(0.0, side * DRIVE_POWER, TO_SCORE_STRAFE_SEC);

        // 3. Shoot the preload.
        shootVolley();

        // 4. Optional pickup cycle and second volley.
        if (ENABLE_PICKUP_CYCLE && opModeIsActive()) {
            status = "pickup";
            driveFor(0.0, side * DRIVE_POWER, PICKUP_STRAFE_SEC);
            setIntake(INTAKE_POWER, INTAKE_POWER);
            driveFor(DRIVE_POWER * 0.6, 0.0, PICKUP_FORWARD_SEC);   // slower so it grabs
            setIntake(0.0, 0.0);
            driveFor(-DRIVE_POWER, 0.0, PICKUP_RETURN_SEC);
            driveFor(0.0, -side * DRIVE_POWER, PICKUP_RETURN_STRAFE_SEC);
            shootVolley();
        }

        // 5. Park. Stop the shooter first so it isn't spinning at the end.
        status = "park";
        setShooterRpm(0.0);
        if (PARK_FORWARD_SEC != 0) {
            driveFor(Math.signum(PARK_FORWARD_SEC) * DRIVE_POWER, 0.0, Math.abs(PARK_FORWARD_SEC));
        }
        if (PARK_STRAFE_SEC > 0) driveFor(0.0, side * DRIVE_POWER, PARK_STRAFE_SEC);

        status = "done";
        stopEverything();
        while (opModeIsActive()) {
            telemetry.addData("Status", status);
            telemetry.addData("Time", "%.1f s", autoTimer.seconds());
            telemetry.update();
        }
    }

    // ----------------------------------------------------------------------------------
    // 3. ACTIONS
    // ----------------------------------------------------------------------------------
    /** Waits for the flywheels, then feeds pollen into them for FEED_SEC. */
    private void shootVolley() {
        status = "waiting for flywheels";
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SPINUP_TIMEOUT_SEC && !shooterAtSpeed()) {
            report();
        }

        status = "shooting";
        innerIntake.setPower(FEED_POWER);
        if (FEED_OUTER_TOO) outerIntake.setPower(INTAKE_POWER);
        waitSeconds(FEED_SEC);
        setIntake(0.0, 0.0);
    }

    /** Drives at a fixed power for a time, then stops. Does nothing for 0 seconds. */
    private void driveFor(double forward, double strafe, double seconds) {
        if (seconds <= 0 || !opModeIsActive()) return;
        drive(forward, strafe, 0.0);
        waitSeconds(seconds);
        drive(0.0, 0.0, 0.0);
    }

    private void waitSeconds(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            report();
        }
    }

    private void report() {
        telemetry.addData("Status", status);
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Inner / Outer RPM", "%.0f / %.0f", rpmOf(innerShooter), rpmOf(outerShooter));
        telemetry.update();
    }

    private void stopEverything() {
        drive(0.0, 0.0, 0.0);
        setShooterRpm(0.0);
        setIntake(0.0, 0.0);
    }

    // ----------------------------------------------------------------------------------
    // 4. MECHANISMS
    // ----------------------------------------------------------------------------------
    private void setIntake(double inner, double outer) {
        innerIntake.setPower(inner);
        outerIntake.setPower(outer);
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
        return Math.abs(rpmOf(innerShooter) - AUTO_SHOOTER_RPM) < SHOOTER_RPM_TOLERANCE
                && Math.abs(rpmOf(outerShooter) - AUTO_SHOOTER_RPM) < SHOOTER_RPM_TOLERANCE;
    }

    /** Points the turret at an angle (degrees clockwise from straight ahead). */
    private void setTurretAngle(double turretDeg) {
        double clipped = Range.clip(turretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
        double sign = TURRET_REVERSED ? -1.0 : 1.0;
        double servoDeg = sign * clipped * TURRET_GEAR_RATIO;
        double pos = Range.clip(0.5 + servoDeg / SERVO_RANGE_DEG, 0.0, 1.0);
        if (turretRight != null) turretRight.setPosition(pos);
        if (turretLeft != null) turretLeft.setPosition(LEFT_SERVO_OPPOSITE ? 1.0 - pos : pos);
    }

    // ----------------------------------------------------------------------------------
    // 5. HARDWARE SETUP
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

        // tryGet returns null instead of crashing, so a missing servo only disables the turret.
        turretRight = hardwareMap.tryGet(Servo.class, TURRET_SERVO_RIGHT_NAME);
        turretLeft = USE_LEFT_TURRET_SERVO
                ? hardwareMap.tryGet(Servo.class, TURRET_SERVO_LEFT_NAME) : null;
    }

    // Drive motor names/directions match pedro/Constants.java and BioBuzzTeleOp.
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
