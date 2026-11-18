package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * ======================================================================================
 *                    TURRET RAW TEST - just proves the turret servo moves
 * ======================================================================================
 * Sets the servo's raw position (0.0 to 1.0) directly. No angles, no gear ratio, no
 * Limelight, no drive. Use it to answer: "does the turret move at all, and which way?"
 *
 *   Left stick X    sweep the position slowly (hold it)
 *   D-pad right     nudge the position up by 0.02       (one press = one step)
 *   D-pad left      nudge the position down by 0.02
 *   A               go to the CENTER (0.50)
 *   X               go to 0.35        B   go to 0.65
 *   Y               switch which servo you are moving (right servo5B <-> left servo0B)
 *
 * SAFETY: the position is kept between MIN_POS and MAX_POS, which are close to the center,
 * so the first test can't slam the turret into a stop. Widen them once you know how far
 * the turret can really go. Stay clear of the turret ring and keep the robot on blocks.
 *
 * IF THE SERVO SPINS INSTEAD OF HOLDING A POSITION: it is probably set to CONTINUOUS
 * ROTATION in Axon Studio. In that mode the "position" is a SPEED (0.5 = stop) and the turret
 * will keep turning. Press A (0.5) to stop it, and tell me, because it needs different code.
 *
 * The telemetry lists every servo the Robot Configuration contains, so you can check
 * the name is spelled the same as the code (servo5B).
 * ======================================================================================
 */
@TeleOp(name = "BioBuzz Turret RAW Test", group = "Test")
public class BioBuzzTurretRawTest extends LinearOpMode {

    private static final String RIGHT_SERVO = "servo5B";
    private static final String LEFT_SERVO = "servo0B";   // not plugged in yet; Y switches to it
    private static final double MIN_POS = 0.30;
    private static final double MAX_POS = 0.70;
    private static final double SWEEP_PER_SEC = 0.15;
    private static final double STEP = 0.02;

    @Override
    public void runOpMode() {
        Servo right = hardwareMap.tryGet(Servo.class, RIGHT_SERVO);
        Servo left = hardwareMap.tryGet(Servo.class, LEFT_SERVO);

        StringBuilder all = new StringBuilder();
        for (Servo s : hardwareMap.getAll(Servo.class)) {
            for (String name : hardwareMap.getNamesOf(s)) all.append(name).append("  ");
        }

        double pos = 0.5;
        boolean useLeft = false;
        boolean prevLeft = false, prevRight = false, prevY = false;
        ElapsedTime clock = new ElapsedTime();

        while (opModeInInit()) {
            telemetry.addData("Right servo (" + RIGHT_SERVO + ")", right != null ? "FOUND" : "NOT FOUND");
            telemetry.addData("Left servo (" + LEFT_SERVO + ")", left != null ? "found" : "not found");
            telemetry.addData("All servos in the config", all.length() == 0 ? "(none)" : all.toString());
            telemetry.addLine("The turret will go to the center (0.50) when you press PLAY.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        clock.reset();

        while (opModeIsActive()) {
            double dt = clock.seconds();
            clock.reset();

            pos += gamepad1.left_stick_x * SWEEP_PER_SEC * dt;
            if (gamepad1.dpad_right && !prevRight) pos += STEP;
            if (gamepad1.dpad_left && !prevLeft) pos -= STEP;
            prevRight = gamepad1.dpad_right;
            prevLeft = gamepad1.dpad_left;
            if (gamepad1.a) pos = 0.5;
            if (gamepad1.x) pos = 0.35;
            if (gamepad1.b) pos = 0.65;
            if (gamepad1.y && !prevY) useLeft = !useLeft;
            prevY = gamepad1.y;
            pos = Range.clip(pos, MIN_POS, MAX_POS);

            Servo active = useLeft ? left : right;
            if (active != null) active.setPosition(pos);

            telemetry.addData("Moving", useLeft ? LEFT_SERVO + " (left)" : RIGHT_SERVO + " (right)");
            telemetry.addData("Servo found", active != null ? "yes" : "NO - nothing can move");
            telemetry.addData("Position", "%.3f   (allowed %.2f to %.2f)", pos, MIN_POS, MAX_POS);
            telemetry.addLine("Stick = sweep | D-pad = nudge | A center | X 0.35 | B 0.65 | Y = other servo");
            telemetry.update();
        }
    }
}
