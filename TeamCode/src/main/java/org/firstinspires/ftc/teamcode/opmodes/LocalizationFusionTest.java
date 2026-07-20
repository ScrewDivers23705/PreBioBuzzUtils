package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode; // Changed from LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// PsiKit Logging Imports
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.List;

@TeleOp(name = "Fusion Localization Test (all-in-one)", group = "Test")
@PsiKitAutoLog // Now aligns perfectly with the underlying lifecycle hooks
public class LocalizationFusionTest extends OpMode {

    private static final double DECISION_MARGIN_THRESHOLD = 50;
    private static final long ASSUMED_LATENCY_MS = 10;

    private TwoWheelLocalizer twoWheelLocalizer;
    private FusionLocalizer fusionLocalizer;
    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        // ---- PsiKit: Record static configuration metadata ----
        Logger.recordMetadata("OpMode", "LocalizationFusionTest");
        Logger.recordMetadata("LocalizationChain", "TwoWheelLocalizer -> FusionLocalizer (Kalman)");
        Logger.recordMetadata("DecisionMarginThreshold", String.valueOf(DECISION_MARGIN_THRESHOLD));
        Logger.recordMetadata("AssumedLatencyMs", String.valueOf(ASSUMED_LATENCY_MS));

        // ---- Odometry setup ----
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, Constants.localizerConstants);

        // ---- Kalman fusion setup ----
        fusionLocalizer = new FusionLocalizer(
                twoWheelLocalizer,
                new Pose(0.25, 0.25, Math.toRadians(2)),
                new Pose(1, 1, Math.toRadians(0.5) / 60),
                new Pose(2.0, 2.0, Math.toRadians(3)),
                100
        );

        // ---- Follower setup ----
        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .mecanumDrivetrain(Constants.driveConstants)
                .pathConstraints(Constants.pathConstraints)
                .setLocalizer(fusionLocalizer)
                .build();

        // ---- AprilTag vision configuration ----
        Position cameraPosition = new Position(DistanceUnit.INCH, 2, 8.6, 13.4, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(685.0826, 681.0935, 364.2377, 245.9037)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90 )));

        telemetry.addData("Status", "Initialized - camera starting up");
        Logger.recordOutput("Status/Phase", "Initialized");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Logger.recordOutput("Status/Phase", "Running");
    }

    @Override
    public void loop() {
        // Runs cyclically, giving the auto-logger defined boundaries for data frames
        follower.update();

        double forwardCmd = -gamepad1.left_stick_y;
        double strafeCmd = -gamepad1.left_stick_x;
        double turnCmd = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forwardCmd, strafeCmd, turnCmd);

        // ---- PsiKit: Log Driver Inputs ----
        Logger.recordOutput("Drive/Cmd/Forward", forwardCmd);
        Logger.recordOutput("Drive/Cmd/Strafe", strafeCmd);
        Logger.recordOutput("Drive/Cmd/Turn", turnCmd);
        Logger.recordOutput("Status/RuntimeSec", getRuntime());

        updateVision();

        Pose fusedPose = fusionLocalizer.getPose();
        Pose rawOdoPose = twoWheelLocalizer.getPose();

        // ---- PsiKit: Log Kalman Fused Coordinate State ----
        Logger.recordOutput("Pose/Fused/Pos", new Pose2d(-(fusedPose.getY()-72) * 0.0254, (fusedPose.getX()-72) * 0.0254, new Rotation2d(Math.toDegrees(fusedPose.getHeading()))));
        Logger.recordOutput("Pose/Fused/X", fusedPose.getX());
        Logger.recordOutput("Pose/Fused/Y", fusedPose.getY());
        Logger.recordOutput("Pose/Fused/HeadingDeg", (fusedPose.getHeading()));

        // ---- PsiKit: Log Raw Odometry Dead-Reckoning State ----
        Logger.recordOutput("Pose/Raw/Pos", new Pose2d(-(rawOdoPose.getY()-72) * 0.0254, (rawOdoPose.getX()-72) * 0.0254, new Rotation2d(Math.toDegrees(rawOdoPose.getHeading()))));
        Logger.recordOutput("Pose/Raw/X", rawOdoPose.getX());
        Logger.recordOutput("Pose/Raw/Y", rawOdoPose.getY());
        Logger.recordOutput("Pose/Raw/HeadingDeg", (rawOdoPose.getHeading()));

        // ---- PsiKit: Log Filter Health Matrix Status ----
        boolean filterNaN = fusionLocalizer.isNAN();
        Logger.recordOutput("Filter/isNAN", filterNaN);

        // Fallback Driver Station Telemetry
        telemetry.addLine("=== KALMAN FUSED STATE ===");
        telemetry.addData("Fused X", "%.2f in", fusedPose.getX());
        telemetry.addData("Fused Y", "%.2f in", fusedPose.getY());
        telemetry.addData("Fused Heading", "%.2f deg", Math.toDegrees(fusedPose.getHeading()));

        telemetry.addLine("=== RAW ODOMETRY ONLY ===");
        telemetry.addData("Raw X", "%.2f in", rawOdoPose.getX());
        telemetry.addData("Raw Y", "%.2f in", rawOdoPose.getY());
        telemetry.addData("Raw Heading", "%.2f deg", Math.toDegrees(rawOdoPose.getHeading()));

        telemetry.addLine("=== FILTER HEALTH ===");
        telemetry.addData("isNAN", filterNaN ? "YES - something is wrong" : "no");
    }

    @Override
    public void stop() {
        // Cleanly tearing down hardware triggers the internal logger file flash/save
        visionPortal.close();
        sleep(250);
        Logger.recordOutput("Status/Phase", "Stopped");
    }

    private void updateVision() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        Logger.recordOutput("Vision/DetectionsCount", detections.size());
        telemetry.addData("AprilTags/detections", detections.size());

        long captureTime = System.nanoTime() - ASSUMED_LATENCY_MS * 1_000_000L;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) continue;

            String tagName = detection.metadata.name;
            Logger.recordOutput("Vision/Tags/" + tagName + "/DecisionMargin", detection.decisionMargin);
            telemetry.addData("AprilTags/" + tagName + " margin", detection.decisionMargin);

            if (detection.decisionMargin <= DECISION_MARGIN_THRESHOLD) continue;
            if (detection.robotPose == null) continue;

            Pose pose = new Pose(
                    detection.robotPose.getPosition().x + 72,
                    -detection.robotPose.getPosition().y + 72,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            );

            Logger.recordOutput("Vision/Tags/" + tagName + "/InputX", pose.getX());
            Logger.recordOutput("Vision/Tags/" + tagName + "/InputY", pose.getY());
            Logger.recordOutput("Vision/Tags/" + tagName + "/InputHeadingDeg", Math.toDegrees(pose.getHeading()));

            fusionLocalizer.addMeasurement(pose, captureTime);
        }
    }
}