package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * ======================================================================================
 *                              LIMELIGHT 3A TELEOP
 * ======================================================================================
 * Drives the robot with mecanum controls and uses a Limelight 3A camera to:
 *   1. show what the camera sees (target angle, size, AprilTag IDs, robot field pose),
 *   2. AUTO-AIM: hold RIGHT BUMPER and the robot turns itself to face the target,
 *   3. switch Limelight pipelines with the D-pad.
 *
 * BEFORE THIS WILL RUN:
 *   - Plug the Limelight into a USB 3.0 port on the Control Hub.
 *   - In the Driver Station Robot Configuration, the Limelight shows up as an
 *     "Ethernet Device". Name it "limelight" (or change LIMELIGHT_NAME below).
 *   - Set up your pipelines at http://limelight.local:5801 (or http://172.29.0.1:5801)
 *     from a laptop plugged into the Limelight / connected to the robot Wi-Fi.
 *     Pipeline 0 = AprilTags is a good place to start.
 *
 * CONTROLS (gamepad 1):
 *   Left stick       drive / strafe
 *   Right stick X    turn
 *   Right bumper     hold to auto-aim at the target
 *   D-pad up/down    next / previous pipeline (0-9)
 *
 * WHAT THE NUMBERS MEAN:
 *   tx  = how far LEFT (-) or RIGHT (+) the target is from the crosshair, in degrees
 *   ty  = how far DOWN (-) or UP (+) the target is from the crosshair, in degrees
 *   ta  = how much of the image the target fills (0-100%). Bigger = closer.
 * ======================================================================================
 */
@TeleOp(name = "Limelight TeleO/p", group = "TeleOp")
public class LimelightTeleOp extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // 1. SETTINGS - tune these on your robot.
    // ----------------------------------------------------------------------------------
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int START_PIPELINE = 0;

    // Auto-aim: turn power = AIM_KP * tx. Too slow -> raise it. Wobbles -> lower it.
    // AIM_DIRECTION: set to 1.0 if the robot turns toward the target, -1.0 if it turns away.
    private static final double AIM_DIRECTION = -1.0;
    private static final double AIM_KP = 0.02;
    private static final double AIM_MIN_POWER = 0.05;   // Enough power to actually move
    private static final double AIM_MAX_POWER = 0.5;    // Don't spin too fast
    private static final double AIM_TOLERANCE_DEG = 1.0; // "Close enough" - stop turning

    // ----------------------------------------------------------------------------------
    // 2. HARDWARE
    // Motor names/directions match pedro/Constants.java. Keep them in sync.
    // ----------------------------------------------------------------------------------
    private Limelight3A limelight;
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private int pipeline = START_PIPELINE;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        initDrive();

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);     // Ask the camera for data 100 times a second
        limelight.pipelineSwitch(pipeline);
        limelight.start();                // MUST be called or you get no results

        telemetry.addLine("Limelight ready. Press PLAY.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handlePipelineButtons();

            LLResult result = limelight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            // ------------------------------------------------------------------------------
            // DRIVING
            // ------------------------------------------------------------------------------
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;  // Counter imperfect strafing
            double turn = gamepad1.right_stick_x;

            if (gamepad1.right_bumper && hasTarget) {
                turn = aimTurnPower(result.getTx());
            }

            drive(forward, strafe, turn);

            // ------------------------------------------------------------------------------
            // TELEMETRY
            // ------------------------------------------------------------------------------
            showStatus();
            if (hasTarget) {
                showResult(result);
            } else {
                telemetry.addLine("No target");
            }
            telemetry.addData("Auto-aim", gamepad1.right_bumper ? "ON" : "off");
            telemetry.update();
        }

        limelight.stop();
    }

    // ----------------------------------------------------------------------------------
    // 3. AUTO-AIM
    // If the target is to the right (tx > 0), turn right; to the left, turn left.
    // The further away from center, the faster we turn.
    // ----------------------------------------------------------------------------------
    private double aimTurnPower(double tx) {
        if (Math.abs(tx) < AIM_TOLERANCE_DEG) return 0.0;

        double power = AIM_DIRECTION * AIM_KP * tx;
        // Make sure there is enough power to overcome friction.
        if (Math.abs(power) < AIM_MIN_POWER) power = Math.copySign(AIM_MIN_POWER, power);
        return Range.clip(power, -AIM_MAX_POWER, AIM_MAX_POWER);
    }

    // ----------------------------------------------------------------------------------
    // 4. PIPELINE SWITCHING (D-pad). Only acts on the press, not while held.
    // ----------------------------------------------------------------------------------
    private void handlePipelineButtons() {
        if (gamepad1.dpad_up && !lastDpadUp && pipeline < 9) {
            pipeline++;
            limelight.pipelineSwitch(pipeline);
        }
        if (gamepad1.dpad_down && !lastDpadDown && pipeline > 0) {
            pipeline--;
            limelight.pipelineSwitch(pipeline);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    // ----------------------------------------------------------------------------------
    // 5. TELEMETRY HELPERS
    // ----------------------------------------------------------------------------------
    private void showStatus() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("Camera", "%.0f fps, %.0f C, CPU %.0f%%",
                status.getFps(), status.getTemp(), status.getCpu());
    }

    private void showResult(LLResult result) {
        telemetry.addData("tx", "%.1f deg", result.getTx());
        telemetry.addData("ty", "%.1f deg", result.getTy());
        telemetry.addData("ta", "%.1f %%", result.getTa());
        telemetry.addData("Data age", "%d ms", result.getStaleness());

        // AprilTags seen (only filled in on an AprilTag / fiducial pipeline).
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {
            telemetry.addData("Tag " + tag.getFiducialId(), "x %.1f, y %.1f deg",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees());
        }

        // Robot position on the field, worked out from the AprilTags (MegaTag 1).
        // Only correct if your field map is uploaded to the Limelight. Units are meters.
        Pose3D botpose = result.getBotpose();
        if (botpose != null && !tags.isEmpty()) {
            telemetry.addData("Botpose", "x %.2f, y %.2f m, yaw %.1f deg",
                    botpose.getPosition().x,
                    botpose.getPosition().y,
                    botpose.getOrientation().getYaw());
        }
    }

    // ----------------------------------------------------------------------------------
    // 6. MECANUM DRIVE
    // ----------------------------------------------------------------------------------
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
        // Scale everything down if any motor would go over 1.0, so the robot keeps
        // driving in the direction you asked instead of curving.
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        leftFront.setPower((forward + strafe + turn) / max);
        leftRear.setPower((forward - strafe + turn) / max);
        rightFront.setPower((forward - strafe - turn) / max);
        rightRear.setPower((forward + strafe - turn) / max);
    }
}
