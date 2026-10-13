package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * ======================================================================================
 *                         BIOBUZZ AUTONOMOUS (Sense and Sound Robotics)
 * ======================================================================================
 * Full 30-second autonomous built on Pedro Pathing 3.0:
 *   1. drives to the scoring spot and scores the preload,
 *   2. runs PICKUP_COUNT cycles of: curve to a pickup spot (intake on) -> drive back
 *      -> score,
 *   3. parks. If time is running low it skips remaining cycles and parks right away.
 *
 * INIT CONTROLS (gamepad 1):
 *   X = BLUE alliance       B = RED alliance
 *   D-pad UP = NEAR start   D-pad DOWN = FAR start
 *
 * BEFORE THIS WILL RUN ON THE ROBOT:
 *   1. Fill in pedro/Constants.java by running the tuners (Foresight is required).
 *   2. Replace every "TODO" pose below with measured BioBuzz field positions.
 *      Draw them at https://visualizer.pedropathing.com first.
 *   3. Fill in the three mechanism methods at the bottom (scoreElement, startIntake,
 *      stopIntake) with the real hardware code.
 *
 * All poses are written for the BLUE alliance. RED is mirrored across x = 72.
 * Heading is in DEGREES: 0 = facing +X, 90 = facing +Y, 180 = facing -X.
 * ======================================================================================
 */
@Autonomous(name = "BioBuzz Autonomous", group = "Autonomous")
public class BioBuzzAuto extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // 1. TUNABLE SETTINGS
    // ----------------------------------------------------------------------------------
    /** Number of pickup -> score cycles after the preload. */
    private static final int PICKUP_COUNT = 2;

    /** Seconds to wait for the scoring mechanism to finish. */
    private static final double SCORE_TIME = 1.0;

    /** Seconds to keep the intake running once we reach a pickup spot. */
    private static final double INTAKE_TIME = 0.75;

    /** Fraction (0-1) of the pickup path at which the intake turns on. */
    private static final double INTAKE_START_COMPLETION = 0.5;

    /** Speed multiplier on pickup paths so the intake can grab the element. */
    private static final double PICKUP_SPEED = 0.5;

    /** Auto length in seconds. */
    private static final double AUTO_LENGTH = 30.0;

    /**
     * Seconds reserved for the trip to park. If less time than this remains when a cycle
     * would start, the robot skips ahead and parks instead.
     */
    private static final double PARK_RESERVE = 5.0;

    // ----------------------------------------------------------------------------------
    // 2. STATE MACHINE
    // ----------------------------------------------------------------------------------
    private enum State {
        DRIVE_TO_SCORE,
        SCORING,
        DRIVE_TO_PICKUP,
        INTAKING,
        DRIVE_BACK_TO_SCORE,
        DRIVE_TO_PARK,
        DONE
    }

    private State state = State.DRIVE_TO_SCORE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    /** How many pickups have been completed so far. */
    private int cyclesDone = 0;
    private boolean intakeRunning = false;

    // ----------------------------------------------------------------------------------
    // 3. FIELD / ALLIANCE SELECTION AND PEDRO OBJECTS
    // ----------------------------------------------------------------------------------
    private Follower follower;
    private boolean redAlliance = false;
    private boolean nearStart = true;

    private Pose startPose, scorePose, parkPose, parkControlPoint;
    private Pose[] pickupPoses, pickupControlPoints;

    private Path scorePreloadPath, parkPath;
    private Path[] grabPaths, returnPaths;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);

        // ------------------------------------------------------------------------------
        // INIT: choose alliance and start spot, then wait for PLAY.
        // ------------------------------------------------------------------------------
        while (opModeInInit()) {
            if (gamepad1.x) redAlliance = false;
            if (gamepad1.b) redAlliance = true;
            if (gamepad1.dpad_up) nearStart = true;
            if (gamepad1.dpad_down) nearStart = false;

            telemetry.addLine("X = BLUE, B = RED | D-pad UP = NEAR, D-pad DOWN = FAR");
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("Start", nearStart ? "NEAR" : "FAR");
            telemetry.addLine("Robot must be at the start pose before pressing PLAY!");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // ------------------------------------------------------------------------------
        // START: build poses and paths for the chosen setup and begin driving.
        // ------------------------------------------------------------------------------
        buildPoses();
        buildPaths();
        follower.setPose(startPose);

        autoTimer.reset();
        follower.follow(scorePreloadPath);
        setState(State.DRIVE_TO_SCORE);

        // ------------------------------------------------------------------------------
        // MAIN LOOP
        // ------------------------------------------------------------------------------
        while (opModeIsActive()) {
            follower.update();   // ALWAYS call this every loop, or the robot won't move.
            updateStateMachine();

            Pose pose = follower.pose();
            telemetry.addData("State", state);
            telemetry.addData("Cycles done", "%d / %d", cyclesDone, PICKUP_COUNT);
            telemetry.addData("Time left", "%.1f s", AUTO_LENGTH - autoTimer.seconds());
            telemetry.addData("X", "%.1f", pose.x());
            telemetry.addData("Y", "%.1f", pose.y());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.heading()));
            telemetry.update();
        }

        // Make sure nothing is left spinning if STOP is pressed mid-intake.
        stopIntake();
    }

    // ----------------------------------------------------------------------------------
    // 4. FIELD POSES
    // TODO: every number in this method is a placeholder. Measure the real BioBuzz
    // positions on the field and replace them, then check that the RED mirror matches.
    // ----------------------------------------------------------------------------------
    private void buildPoses() {
        PoseFactory field = PoseFactory.degrees();
        if (redAlliance) field = field.mirrorX(72);

        if (nearStart) {
            startPose = field.of(9, 60, 0);     // TODO: NEAR start, against the wall
            scorePose = field.of(36, 72, 0);    // TODO: NEAR scoring spot
        } else {
            startPose = field.of(9, 24, 0);     // TODO: FAR start, against the wall
            scorePose = field.of(36, 36, 0);    // TODO: FAR scoring spot
        }

        // Pickup spots, in the order they are visited. Need PICKUP_COUNT entries.
        pickupPoses = new Pose[] {
                field.of(28, 36, 90),           // TODO: first pickup
                field.of(28, 12, 90)            // TODO: second pickup
        };
        // Control points pull the curves toward them (their heading is ignored).
        pickupControlPoints = new Pose[] {
                field.of(44, 50, 0),            // TODO
                field.of(44, 30, 0)             // TODO
        };

        parkControlPoint = field.of(60, 40, 0); // TODO
        parkPose = field.of(60, 12, 270);       // TODO: parking zone
    }

    // ----------------------------------------------------------------------------------
    // 5. PATHS
    // ----------------------------------------------------------------------------------
    private void buildPaths() {
        if (pickupPoses.length < PICKUP_COUNT || pickupControlPoints.length < PICKUP_COUNT) {
            throw new IllegalStateException(
                    "PICKUP_COUNT is " + PICKUP_COUNT + " but fewer pickup poses were defined.");
        }

        scorePreloadPath = line(startPose, scorePose)
                .linear(startPose, scorePose);

        grabPaths = new Path[PICKUP_COUNT];
        returnPaths = new Path[PICKUP_COUNT];
        for (int i = 0; i < PICKUP_COUNT; i++) {
            // Slower on the way to the pickup so the intake can grab the element.
            grabPaths[i] = curve(scorePose, pickupControlPoints[i], pickupPoses[i])
                    .linear(scorePose, pickupPoses[i])
                    .with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED));

            returnPaths[i] = curve(pickupPoses[i], pickupControlPoints[i], scorePose)
                    .linear(pickupPoses[i], scorePose);
        }

        parkPath = curve(scorePose, parkControlPoint, parkPose)
                .linear(scorePose, parkPose);
    }

    // ----------------------------------------------------------------------------------
    // 6. STATE MACHINE
    // "!follower.isBusy()" means the robot reached the end of its path and stopped.
    // ----------------------------------------------------------------------------------
    private void updateStateMachine() {
        switch (state) {
            case DRIVE_TO_SCORE:
                if (!follower.isBusy()) {
                    scoreElement();
                    setState(State.SCORING);
                }
                break;

            case SCORING:
                if (stateTimer.seconds() > SCORE_TIME) {
                    startNextCycleOrPark();
                }
                break;

            case DRIVE_TO_PICKUP:
                // Turn the intake on partway along the path, while still driving.
                if (!intakeRunning && follower.completion() > INTAKE_START_COMPLETION) {
                    startIntake();
                }
                if (!follower.isBusy()) {
                    setState(State.INTAKING);
                }
                break;

            case INTAKING:
                if (stateTimer.seconds() > INTAKE_TIME) {
                    stopIntake();
                    follower.follow(returnPaths[cyclesDone]);
                    setState(State.DRIVE_BACK_TO_SCORE);
                }
                break;

            case DRIVE_BACK_TO_SCORE:
                if (!follower.isBusy()) {
                    cyclesDone++;
                    scoreElement();
                    setState(State.SCORING);
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;

            case DONE:
                // The follower keeps holding the park pose so we don't get pushed.
                break;
        }
    }

    /**
     * Starts the next pickup cycle if there are cycles left AND enough time remains to
     * finish it and still park. Otherwise heads to the park spot.
     */
    private void startNextCycleOrPark() {
        boolean cyclesLeft = cyclesDone < PICKUP_COUNT;
        boolean timeLeft = AUTO_LENGTH - autoTimer.seconds() > PARK_RESERVE + cycleEstimate();

        if (cyclesLeft && timeLeft) {
            follower.follow(grabPaths[cyclesDone]);
            setState(State.DRIVE_TO_PICKUP);
        } else {
            follower.follow(parkPath);
            setState(State.DRIVE_TO_PARK);
        }
    }

    /**
     * Rough seconds one full pickup cycle takes. TODO: time a real cycle on the robot and
     * update this number so the "skip to park" check is accurate.
     */
    private double cycleEstimate() {
        return 8.0;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    // ----------------------------------------------------------------------------------
    // 7. MECHANISMS
    // TODO: replace these with the robot's real intake / scorer code. They are empty so
    // this auto runs on any drivetrain without crashing.
    // ----------------------------------------------------------------------------------
    private void scoreElement() {
        // Example: scorer.setPosition(SCORER_OPEN);
    }

    private void startIntake() {
        intakeRunning = true;
        // Example: intake.setPower(1.0);
    }

    private void stopIntake() {
        intakeRunning = false;
        // Example: intake.setPower(0.0);
    }
}
