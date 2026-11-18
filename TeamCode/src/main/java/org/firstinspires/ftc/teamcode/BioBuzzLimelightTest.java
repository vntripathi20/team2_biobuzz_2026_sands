package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ======================================================================================
 *                LIMELIGHT TEST - prints exactly what the Limelight reports
 * ======================================================================================
 * No drive, no turret, no shooter. It only reads the camera, so you can see whether the
 * robot code receives AprilTags at all.
 *
 *   D-pad up / down    switch the Limelight to the next / previous pipeline slot (0 to 9)
 *
 * Hold a tag in front of the camera and read the screen:
 *   "Valid"           should say true when a tag is in view.
 *   "Pipeline type"   should name a fiducial/AprilTag type.
 *   "Tags"            lists the IDs it sees, with tx (left/right angle in degrees).
 * Tell Claude what those lines say. The slot your AprilTag pipeline is in should be the
 * one the screen is on. Start at slot 3 (D-pad up/down to change).
 * ======================================================================================
 */
@TeleOp(name = "BioBuzz Limelight Test", group = "Test")
public class BioBuzzLimelightTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int START_PIPELINE = 3;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        if (limelight == null) {
            while (opModeInInit() && !isStopRequested()) {
                telemetry.addLine("NO Limelight named \"" + LIMELIGHT_NAME + "\" in the Robot Configuration");
                telemetry.update();
            }
            return;
        }

        int pipeline = START_PIPELINE;
        boolean prevUp = false, prevDown = false;

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        while (opModeInInit()) {
            telemetry.addLine("Limelight test. Press PLAY, then hold a tag in view.");
            telemetry.addData("Connected", limelight.isConnected());
            telemetry.update();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !prevUp && pipeline < 9) {
                pipeline++;
                limelight.pipelineSwitch(pipeline);
            }
            if (gamepad1.dpad_down && !prevDown && pipeline > 0) {
                pipeline--;
                limelight.pipelineSwitch(pipeline);
            }
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            telemetry.addData("Asked for slot", pipeline);
            telemetry.addData("Connected / running", limelight.isConnected() + " / " + limelight.isRunning());
            telemetry.addData("Status pipeline index", status.getPipelineIndex());
            telemetry.addData("Status FPS", "%.1f", status.getFps());
            telemetry.addData("Since last update (ms)", limelight.getTimeSinceLastUpdate());

            if (result == null) {
                telemetry.addLine("Result: NULL (nothing received yet)");
            } else {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Result pipeline", result.getPipelineIndex() + " (" + result.getPipelineType() + ")");
                telemetry.addData("Staleness (ms)", result.getStaleness());
                telemetry.addData("Capture latency (ms)", "%.1f", result.getCaptureLatency());
                StringBuilder tags = new StringBuilder();
                for (LLResultTypes.FiducialResult t : result.getFiducialResults()) {
                    tags.append("ID ").append(t.getFiducialId())
                            .append(String.format(" tx %.1f   ", t.getTargetXDegrees()));
                }
                telemetry.addData("Tags (" + result.getFiducialResults().size() + ")",
                        tags.length() == 0 ? "none" : tags.toString());
                telemetry.addData("Other targets", "detector %d, color %d, classifier %d",
                        result.getDetectorResults().size(), result.getColorResults().size(),
                        result.getClassifierResults().size());
            }
            telemetry.addLine("D-pad up/down = change slot");
            telemetry.update();
        }
        limelight.stop();
    }
}
