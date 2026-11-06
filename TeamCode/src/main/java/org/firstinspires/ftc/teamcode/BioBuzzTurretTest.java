package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TURRET TEST: moves only the turret servo so you can check it works. No drive, no shooter,
 * no Limelight. Point the turret straight ahead before pressing INIT.
 *
 *   Right stick X   sweep the turret left/right (hold to keep moving)
 *   D-pad left      jump to -30 deg        D-pad right   jump to +30 deg
 *   D-pad up        jump to +60 deg        D-pad down    jump to -60 deg
 *   A               back to center (0)
 *   X               hold: slow speed       (default speed is medium)
 *
 * Only the RIGHT servo (servo5B) is used until the left one is wired. Watch the
 * telemetry "Servo position" and "Turret angle": if it moves the wrong way, flip REVERSED.
 */
@TeleOp(name = "BioBuzz Turret Test", group = "Test")
public class BioBuzzTurretTest extends LinearOpMode {

    private static final String RIGHT_SERVO = "servo5B";
    private static final String LEFT_SERVO = "servo0B";
    private static final boolean USE_LEFT = false;        // leave false until it's plugged in
    private static final boolean LEFT_OPPOSITE = true;    // flip if the two servos fight
    private static final boolean REVERSED = false;        // flip if positive moves the wrong way
    private static final double GEAR_RATIO = 1.0;         // TODO: turret teeth / servo gear teeth
    private static final double SERVO_RANGE_DEG = 355.0;  // TODO: servo travel for 0 -> 1
    private static final double MIN_DEG = -90.0;          // TODO: safe limits
    private static final double MAX_DEG = 90.0;
    private static final double SWEEP_DEG_PER_SEC = 60.0;
    private static final double SLOW_DEG_PER_SEC = 20.0;

    @Override
    public void runOpMode() {
        Servo right = hardwareMap.tryGet(Servo.class, RIGHT_SERVO);
        Servo left = USE_LEFT ? hardwareMap.tryGet(Servo.class, LEFT_SERVO) : null;

        double angle = 0.0;
        ElapsedTime clock = new ElapsedTime();

        while (opModeInInit()) {
            telemetry.addData("Right servo (servo5B)", right != null ? "found" : "NOT FOUND");
            telemetry.addLine("Point the turret straight ahead, then press PLAY");
            telemetry.update();
        }
        if (isStopRequested()) return;
        clock.reset();

        while (opModeIsActive()) {
            double dt = clock.seconds();
            clock.reset();

            double speed = gamepad1.x ? SLOW_DEG_PER_SEC : SWEEP_DEG_PER_SEC;
            angle += gamepad1.right_stick_x * speed * dt;
            if (gamepad1.dpad_left) angle = -30;
            if (gamepad1.dpad_right) angle = 30;
            if (gamepad1.dpad_up) angle = 60;
            if (gamepad1.dpad_down) angle = -60;
            if (gamepad1.a) angle = 0;
            angle = Range.clip(angle, MIN_DEG, MAX_DEG);

            double servoDeg = (REVERSED ? -1.0 : 1.0) * angle * GEAR_RATIO;
            double pos = Range.clip(0.5 + servoDeg / SERVO_RANGE_DEG, 0.0, 1.0);
            if (right != null) right.setPosition(pos);
            if (left != null) left.setPosition(LEFT_OPPOSITE ? 1.0 - pos : pos);

            telemetry.addData("Turret angle", "%.1f deg", angle);
            telemetry.addData("Servo position", "%.3f", pos);
            telemetry.addData("Right servo", right != null ? "found" : "NOT FOUND");
            telemetry.addData("Left servo", USE_LEFT ? (left != null ? "found" : "NOT FOUND") : "off");
            telemetry.update();
        }
    }
}
