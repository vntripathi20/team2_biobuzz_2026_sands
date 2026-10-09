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

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.List;

/**
 * ======================================================================================
 *                              LIMELIGHT 3A TELEOP
 * ======================================================================================
 * Drives the robot with mecanum controls and uses a Limelight 3A camera to:
 *   1. show what the camera sees (target angle, size, AprilTag IDs, robot field pose),
 *   2. FOLLOW: hold RIGHT BUMPER and the robot turns to face the target AND drives
 *      toward / away from it to stay at a set distance (you can still strafe),
 *   3. stream the Limelight camera to the Driver Station (see CAMERA FEED below),
 *   4. switch Limelight pipelines with the D-pad.
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
 *   Right bumper     hold to follow the target
 *   D-pad up/down    next / previous pipeline (0-9)
 *
 * CAMERA FEED:
 *   Telemetry can only hold text, so the feed goes to the Driver Station's camera stream.
 *   After INIT, tap the three dots (top right) -> "Camera Stream". Needs the Limelight on
 *   its default IP (172.29.0.1, USB). Change LIMELIGHT_STREAM_URL if yours differs.
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
    private static final String LIMELIGHT_STREAM_URL = "http://172.29.0.1:5800";

    // Auto-aim: turn power = AIM_KP * tx. Too slow -> raise it. Wobbles -> lower it.
    // AIM_DIRECTION: set to 1.0 if the robot turns toward the target, -1.0 if it turns away.
    private static final double AIM_DIRECTION = -1.0;
    private static final double AIM_KP = 0.02;
    private static final double AIM_MIN_POWER = 0.05;   // Enough power to actually move
    private static final double AIM_MAX_POWER = 0.5;    // Don't spin too fast
    private static final double AIM_TOLERANCE_DEG = 1.0; // "Close enough" - stop turning

    // Follow: forward power = FOLLOW_KP * (FOLLOW_TARGET_AREA - ta). ta is how much of the
    // image the target fills, so a bigger ta means closer. Set FOLLOW_TARGET_AREA to the
    // ta you see in telemetry at the distance you want to hold.
    // FOLLOW_DIRECTION: set to -1.0 if the robot backs away when it should come closer.
    private static final double FOLLOW_DIRECTION = 1.0;
    private static final double FOLLOW_TARGET_AREA = 5.0;  // percent of image
    private static final double FOLLOW_KP = 0.08;
    private static final double FOLLOW_TOLERANCE = 0.3;    // percent - "close enough"
    private static final double FOLLOW_MAX_POWER = 0.5;

    // ----------------------------------------------------------------------------------
    // 2. HARDWARE
    // Motor names/directions match pedro/Constants.java. Keep them in sync.
    // ----------------------------------------------------------------------------------
    private Limelight3A limelight;
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private final LimelightStream stream = new LimelightStream(LIMELIGHT_STREAM_URL);

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

        stream.start();
        CameraStreamServer.getInstance().setSource(stream);

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
                forward = followForwardPower(result.getTa());
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
            telemetry.addData("Follow", gamepad1.right_bumper ? "ON" : "off");
            telemetry.addData("Camera feed", stream.isStreaming()
                    ? "live - Driver Station menu > Camera Stream" : "not connected");
            telemetry.update();
        }

        stream.stop();
        CameraStreamServer.getInstance().setSource(null);
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

    // Drive toward the target when it looks too small (far), back off when too big (close).
    private double followForwardPower(double ta) {
        double error = FOLLOW_TARGET_AREA - ta;
        if (Math.abs(error) < FOLLOW_TOLERANCE) return 0.0;
        return Range.clip(FOLLOW_DIRECTION * FOLLOW_KP * error, -FOLLOW_MAX_POWER, FOLLOW_MAX_POWER);
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

    // ----------------------------------------------------------------------------------
    // 7. CAMERA STREAM
    // Reads the Limelight's MJPEG stream on a background thread and keeps the newest frame
    // so the Driver Station's "Camera Stream" view can show it.
    // ----------------------------------------------------------------------------------
    private static class LimelightStream implements CameraStreamSource {
        private final String url;
        private volatile boolean running = false;
        private volatile boolean streaming = false;
        private volatile Bitmap latest;
        private Thread thread;

        LimelightStream(String url) {
            this.url = url;
        }

        boolean isStreaming() {
            return streaming;
        }

        void start() {
            running = true;
            thread = new Thread(this::readLoop, "LimelightStream");
            thread.setDaemon(true);
            thread.start();
        }

        void stop() {
            running = false;
            if (thread != null) thread.interrupt();
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            final Bitmap frame = latest;
            if (frame == null) return;
            continuation.dispatch(consumer -> consumer.accept(frame));
        }

        // Pulls JPEG images out of the stream by finding their start (FF D8) and end (FF D9).
        private void readLoop() {
            while (running) {
                HttpURLConnection conn = null;
                try {
                    conn = (HttpURLConnection) new URL(url).openConnection();
                    conn.setConnectTimeout(2000);
                    conn.setReadTimeout(2000);
                    InputStream in = new BufferedInputStream(conn.getInputStream(), 64 * 1024);
                    ByteArrayOutputStream buf = new ByteArrayOutputStream();
                    boolean inImage = false;
                    int prev = -1, b;
                    while (running && (b = in.read()) != -1) {
                        if (!inImage) {
                            if (prev == 0xFF && b == 0xD8) {
                                buf.reset();
                                buf.write(0xFF);
                                buf.write(0xD8);
                                inImage = true;
                            }
                        } else {
                            buf.write(b);
                            if (prev == 0xFF && b == 0xD9) {
                                byte[] jpg = buf.toByteArray();
                                Bitmap bmp = BitmapFactory.decodeByteArray(jpg, 0, jpg.length);
                                if (bmp != null) {
                                    latest = bmp;
                                    streaming = true;
                                }
                                inImage = false;
                            }
                        }
                        prev = b;
                    }
                } catch (Exception e) {
                    // Camera not reachable yet or connection dropped - retry below.
                } finally {
                    streaming = false;
                    if (conn != null) conn.disconnect();
                }
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }
}
