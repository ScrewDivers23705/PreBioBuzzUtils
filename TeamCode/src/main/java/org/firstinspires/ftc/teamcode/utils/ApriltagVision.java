package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.pedropathing.localization.Localizer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Owns both the AprilTag camera pipeline and the FusionLocalizer that blends it with
 * whatever dead-reckoning Localizer you pass in (your TwoWheelLocalizer). No Context,
 * no other subsystem classes required - just plain HardwareMap and Telemetry, so it
 * drops straight into a bare OpMode.
 *
 * Usage:
 *   AprilTagLocalizer aprilTags = new AprilTagLocalizer(
 *       hardwareMap, telemetry, twoWheelLocalizer, "Webcam 1",
 *       cameraPosition, cameraOrientation, fx, fy, cx, cy
 *   );
 *
 *   Follower follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
 *       .mecanumDrivetrain(Constants.driveConstants)
 *       .pathConstraints(Constants.pathConstraints)
 *       .setLocalizer(aprilTags.getLocalizer())
 *       .build();
 *
 *   // every loop:
 *   follower.update();
 *   aprilTags.update();
 */
public final class ApriltagVision implements AutoCloseable {

    public static int latencyMs = 10;
    public static double decisionMarginThreshold = 50;

    private final Telemetry telemetry;
    private final AprilTagProcessor processor;
    //private final FusionLocalizer fusion;
    private final VisionPortal visionPortal;

    /**
     * @param hardwareMap        standard OpMode hardwareMap
     * @param telemetry          standard OpMode telemetry, for detection/margin logging
     * @param deadReckoning      your TwoWheelLocalizer (or any other Localizer) providing
     *                           the predict-step odometry that vision corrects
     * @param webcamName         name configured on the Robot Controller (e.g. "Webcam 1")
     * @param cameraPositionOnRobot    camera's position relative to your robot's tracking origin
     * @param cameraOrientationOnRobot camera's orientation relative to your robot
     * @param fx,fy,cx,cy        lens intrinsics from your camera's calibration
     */
    public ApriltagVision(HardwareMap hardwareMap, Telemetry telemetry, Localizer deadReckoning,
                             String webcamName,
                             Position cameraPositionOnRobot, YawPitchRollAngles cameraOrientationOnRobot,
                             double fx, double fy, double cx, double cy) {
        this.telemetry = telemetry;

        // Starting point only - tune using the drift-then-correct test: jittery when a
        // tag is visible means measurement variance (3rd arg) is too small; barely
        // reacting to a clearly good tag read means it's too large, or process noise
        // (2nd arg) is too small.

        /*fusion = new FusionLocalizer(
                deadReckoning,
                new Pose(0.25, 0.25, Math.toRadians(2)),   // initial uncertainty
                new Pose(1, 1, Math.toRadians(0.5) / 60),  // process noise
                new Pose(2.0, 2.0, Math.toRadians(3)),     // default vision variance
                100
        );*/

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

    /*public Localizer getLocalizer() {
        return fusion;
    }*/

    /** Call once per loop, after your Follower's own update(). */
    public void update() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return; // camera still starting up - nothing usable yet
        }

        List<AprilTagDetection> detections = processor.getDetections();
        telemetry.addData("AprilTags/detections", detections.size());

        long captureTime = System.nanoTime() - latencyMs * 1_000_000L;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) continue;
            telemetry.addData("AprilTags/" + detection.metadata.name + " margin", detection.decisionMargin);
            if (detection.decisionMargin <= decisionMarginThreshold) continue;
            if (detection.robotPose == null) continue;

            // Field-coordinate to Pedro-coordinate conversion - verify against your own
            // field/season by placing the robot at a known pose next to a tag and
            // comparing, as discussed: this offset assumes a standard 144" field with
            // your Pedro origin at a corner.

            /*Pose pose = new Pose(
                    detection.robotPose.getPosition().y + 72,
                    -detection.robotPose.getPosition().x + 72,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            );

            fusion.addMeasurement(pose, captureTime); */
        }
    }

    @Override
    public void close() {
        visionPortal.close();
    }
}