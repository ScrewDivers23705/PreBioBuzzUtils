package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Owns the AprilTag camera pipeline and feeds accepted detections into a
 * KalmanFusionLocalizer. Call update(fusion) once per loop, after your
 * fusion localizer's own update() - detections are filtered by confidence
 * and tag ID, converted into Pedro's pose convention, and given a
 * distance-scaled measurement variance before being fused.
 */
public final class ApriltagVision {

    // --- FTC Dashboard-tunable thresholds ---
    public static double decisionMarginThreshold = 50;   // reject low-confidence detections below this
    public static double baseVisionVarianceAt12in = 0.5; // measurement variance (in^2) at close range
    public static long assumedLatencyMs = 10;            // camera + pipeline delay to timestamp-correct for

    private final AprilTagProcessor processor;
    private final VisionPortal visionPortal;
    private final Set<Integer> acceptedTagIds; // null = accept every tag in your library

    /**
     * @param hardwareMap               standard OpMode hardwareMap
     * @param webcamName                name configured on the Robot Controller
     * @param cameraPositionOnRobot     camera's position relative to the robot's tracking origin
     * @param cameraOrientationOnRobot  camera's orientation relative to the robot
     * @param fx,fy,cx,cy               lens intrinsics from your camera calibration
     * @param acceptedTagIds            restrict fusion to these tag IDs (e.g. only field-fixed tags,
     *                                  excluding any game-piece markers), or null to accept all.
     *                                  Pass a HashSet, not a List - this is checked on every detection
     *                                  every loop, and HashSet.contains() is O(1) versus a list scan.
     */
    public ApriltagVision(HardwareMap hardwareMap, String webcamName,
                                   Position cameraPositionOnRobot, YawPitchRollAngles cameraOrientationOnRobot,
                                   double fx, double fy, double cx, double cy,
                                   Set<Integer> acceptedTagIds) {
        this.acceptedTagIds = acceptedTagIds == null ? null : new HashSet<>(acceptedTagIds);

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPositionOnRobot, cameraOrientationOnRobot)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    /** Call once per loop. Filters detections and fuses any that pass. */
    public void update(FusionLocalizer fusion) {
        update(fusion, null);
    }

    /** Same as update(fusion), with optional telemetry for tuning decisionMarginThreshold live. */
    public void update(FusionLocalizer fusion, Telemetry telemetry) {
        // Skip entirely while the camera is still starting up - avoids doing work on
        // a portal that isn't ready and avoids acting on stale/garbage first frames.
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        List<AprilTagDetection> detections = processor.getDetections();
        if (detections.isEmpty()) return;

        long captureTime = System.nanoTime() - assumedLatencyMs * 1_000_000L;
        if (telemetry != null) telemetry.addData("AprilTags/detections", detections.size());

        for (AprilTagDetection detection : detections) {
            if (!isUsable(detection)) continue;

            if (telemetry != null) {
                telemetry.addData("AprilTags/" + detection.metadata.name + " margin", detection.decisionMargin);
            }

            Pose pose = toPedroPose(detection);
            double distanceInches = detection.ftcPose.range;
            double variance = varianceFromDistance(distanceInches);

            fusion.addMeasurement(
                    pose,
                    captureTime,
                    new Pose(variance, variance, variance * 2)); // heading typically noisier than xy from a tag
        }
    }

    /**
     * Guards against every way a single bad or partial detection could otherwise crash
     * the loop or feed garbage into the filter: missing tag metadata, a tag outside your
     * accepted set, a low-confidence read, or a detection whose pose fields didn't
     * actually resolve (can happen transiently even when metadata is present).
     */
    private boolean isUsable(AprilTagDetection detection) {
        if (detection.metadata == null) return false;
        if (acceptedTagIds != null && !acceptedTagIds.contains(detection.id)) return false;
        if (detection.decisionMargin <= decisionMarginThreshold) return false;
        if (detection.robotPose == null || detection.ftcPose == null) return false;
        return true;
    }

    /**
     * Converts an AprilTag detection's field-frame robot pose into Pedro's coordinate
     * convention. THIS MAPPING IS SPECIFIC TO YOUR FIELD/ROBOT SETUP - the axis order,
     * signs, and any origin offset here depend on how your tag library's field coordinates
     * relate to Pedro's origin and axis directions, which differ by season/setup.
     * <p>
     * To derive your own version: place the robot at a known Pedro pose next to a visible
     * tag, log both fusion.getPose() (before any vision correction) and detection.robotPose
     * side by side, and solve for the rotation/offset that makes them agree. Repeat at a
     * second, different pose to confirm.
     */
    private Pose toPedroPose(AprilTagDetection detection) {
        // Placeholder identity-ish mapping - replace with your derived transform.
        double x = detection.robotPose.getPosition().x;
        double y = detection.robotPose.getPosition().y;
        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        return PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH,x, y, AngleUnit.RADIANS,heading),InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    /**
     * Grows measurement variance with distance, since AprilTag pose accuracy degrades
     * the farther and more oblique the tag is. Tune the reference distance/exponent by
     * logging real detection jitter at a few known distances and fitting to it.
     */
    private static double varianceFromDistance(double distanceInches) {
        double referenceDistance = 12.0;
        double scale = distanceInches / referenceDistance;
        return baseVisionVarianceAt12in * (1 + scale * scale);
    }

    public void close() {
        visionPortal.close();
    }
}