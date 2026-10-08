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
 *                          SAMPLE PEDRO PATHING AUTONOMOUS OPMODE
 * ======================================================================================
 * A complete, working autonomous built on Pedro Pathing 3.0. It:
 *   1. drives to a scoring spot and scores the preloaded game element,
 *   2. curves over to pick up another element (slowing down while intaking),
 *   3. drives back and scores it,
 *   4. parks.
 *
 * BEFORE THIS WILL RUN: fill in pedro/Constants.java by running the tuners.
 * (See the comment at the top of Constants.java.)
 *
 * HOW PEDRO PATHING WORKS:
 * 1. Pose:     where the robot is - X and Y in inches, plus the direction it faces.
 *              The field is 144 x 144 inches, with (0, 0) in one corner.
 * 2. Path:     a line or smooth Bezier curve between poses. You also choose how the
 *              robot turns along the way (constant, linear, or tangent heading).
 * 3. Follower: call follower.update() EVERY loop. It reads the odometry and sets the
 *              motor powers to keep the robot on the path.
 * 4. State machine: the auto is split into steps. Each step waits for something
 *              (path finished, timer done) and then starts the next step.
 *
 * TIP: draw your paths at https://visualizer.pedropathing.com before typing in numbers.
 * ======================================================================================
 */
@Autonomous(name = "Sample Pedro Pathing Autonomous", group = "Autonomous")
public class SamplePedroPathingAuto extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // 1. TIMINGS (seconds) - how long to wait for mechanisms. Tune these on your robot.
    // ----------------------------------------------------------------------------------
    private static final double SCORE_TIME = 1.0;
    private static final double INTAKE_TIME = 0.75;

    // ----------------------------------------------------------------------------------
    // 2. STATE MACHINE STEPS
    // ----------------------------------------------------------------------------------
    private enum State {
        DRIVE_TO_SCORE,
        SCORE_PRELOAD,
        DRIVE_TO_PICKUP,
        INTAKE,
        DRIVE_BACK_TO_SCORE,
        SCORE_PICKUP,
        DRIVE_TO_PARK,
        DONE
    }

    private State state = State.DRIVE_TO_SCORE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ----------------------------------------------------------------------------------
    // 3. PEDRO PATHING OBJECTS
    // ----------------------------------------------------------------------------------
    private Follower follower;
    private boolean redAlliance = false;

    private Pose startPose, scorePose, pickupPose, pickupControlPoint, parkControlPoint, parkPose;
    private Path scorePreloadPath, grabPickupPath, scorePickupPath, parkPath;

    @Override
    public void runOpMode() {
        // Build the follower from the settings in Constants.java.
        follower = Constants.create(hardwareMap);

        // ------------------------------------------------------------------------------
        // INIT: pick the alliance with gamepad 1 while waiting for PLAY.
        // ------------------------------------------------------------------------------
        while (opModeInInit()) {
            if (gamepad1.x) redAlliance = false;
            if (gamepad1.b) redAlliance = true;

            telemetry.addLine("Press X for BLUE, B for RED");
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addLine("Robot must be at the start pose before pressing PLAY!");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // ------------------------------------------------------------------------------
        // START: build the paths for the chosen alliance, tell the follower where the
        // robot is, and start the first path.
        // ------------------------------------------------------------------------------
        buildPoses();
        buildPaths();
        follower.setPose(startPose);

        follower.follow(scorePreloadPath);
        setState(State.DRIVE_TO_SCORE);

        // ------------------------------------------------------------------------------
        // MAIN LOOP: runs until the 30 seconds are up or STOP is pressed.
        // ------------------------------------------------------------------------------
        while (opModeIsActive()) {
            follower.update();   // ALWAYS call this every loop, or the robot won't move.
            updateStateMachine();

            Pose pose = follower.pose();
            telemetry.addData("State", state);
            telemetry.addData("X", "%.1f", pose.x());
            telemetry.addData("Y", "%.1f", pose.y());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.heading()));
            telemetry.addData("Path busy", follower.isBusy());
            telemetry.update();
        }
    }

    // ----------------------------------------------------------------------------------
    // 4. FIELD POSES
    // Write every pose as if you are on the BLUE alliance. For RED, the factory
    // mirrors them across the middle of the field (x = 72) automatically.
    // Heading is in DEGREES: 0 = facing +X, 90 = facing +Y, 180 = facing -X.
    //
    // TODO: these are example spots. Measure the real BioBuzz field locations
    // and replace them. Check on the field that mirroring matches your setup.
    // ----------------------------------------------------------------------------------
    private void buildPoses() {
        PoseFactory field = PoseFactory.degrees();
        if (redAlliance) field = field.mirrorX(72);

        startPose          = field.of(9, 60, 0);     // Against the wall, center of robot
        scorePose          = field.of(36, 72, 0);    // Where we score
        pickupPose         = field.of(28, 36, 90);   // Where a game element is waiting
        pickupControlPoint = field.of(44, 50, 0);    // Pulls the curve - heading is ignored
        parkControlPoint   = field.of(60, 40, 0);
        parkPose           = field.of(60, 12, 270);  // Parking zone
    }

    // ----------------------------------------------------------------------------------
    // 5. PATHS
    // line(a, b)          -> straight line from a to b
    // curve(a, c, b)      -> smooth curve from a to b, bent toward control point c
    //
    // Then pick how the robot turns while driving:
    //   .constant(pose)   -> keep facing one direction the whole way
    //   .linear(a, b)     -> turn smoothly from a's heading to b's heading
    //   .tangent()        -> face the direction of travel (like a car)
    // ----------------------------------------------------------------------------------
    private void buildPaths() {
        scorePreloadPath = line(startPose, scorePose)
                .linear(startPose, scorePose);

        // Drive at half speed on this one so the intake can grab the element.
        // "maxPathSpeed.at(0.5)" only applies while this path is running.
        grabPickupPath = curve(scorePose, pickupControlPoint, pickupPose)
                .linear(scorePose, pickupPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        scorePickupPath = curve(pickupPose, pickupControlPoint, scorePose)
                .linear(pickupPose, scorePose);

        parkPath = curve(scorePose, parkControlPoint, parkPose)
                .linear(scorePose, parkPose);
    }

    // ----------------------------------------------------------------------------------
    // 6. STATE MACHINE
    // "!follower.isBusy()" means the robot reached the end of its path and stopped.
    // Each step starts the next path BEFORE switching state, so there are no gaps.
    // ----------------------------------------------------------------------------------
    private void updateStateMachine() {
        switch (state) {
            case DRIVE_TO_SCORE:
                if (!follower.isBusy()) {
                    scoreElement();
                    setState(State.SCORE_PRELOAD);
                }
                break;

            case SCORE_PRELOAD:
                if (stateTimer.seconds() > SCORE_TIME) {
                    follower.follow(grabPickupPath);
                    setState(State.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:
                // You can run mechanisms DURING a path. Here the intake turns on when
                // the robot is halfway along the path.
                if (follower.completion() > 0.5) {
                    startIntake();
                }
                if (!follower.isBusy()) {
                    setState(State.INTAKE);
                }
                break;

            case INTAKE:
                if (stateTimer.seconds() > INTAKE_TIME) {
                    stopIntake();
                    follower.follow(scorePickupPath);
                    setState(State.DRIVE_BACK_TO_SCORE);
                }
                break;

            case DRIVE_BACK_TO_SCORE:
                if (!follower.isBusy()) {
                    scoreElement();
                    setState(State.SCORE_PICKUP);
                }
                break;

            case SCORE_PICKUP:
                if (stateTimer.seconds() > SCORE_TIME) {
                    follower.follow(parkPath);
                    setState(State.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;

            case DONE:
                // The follower keeps holding the park pose, so the robot won't get pushed.
                break;
        }
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    // ----------------------------------------------------------------------------------
    // 7. MECHANISMS
    // TODO: replace these with your robot's real arm / claw / intake code. They are
    // empty so this sample runs on any drivetrain.
    // ----------------------------------------------------------------------------------
    private void scoreElement() {
        // Example: claw.setPosition(CLAW_OPEN);
    }

    private void startIntake() {
        // Example: intake.setPower(1.0);
    }

    private void stopIntake() {
        // Example: intake.setPower(0.0);
    }
}
