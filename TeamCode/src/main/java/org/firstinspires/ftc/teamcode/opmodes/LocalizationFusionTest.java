package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.utils.ApriltagVision;

@TeleOp(name = "Localization Fusion Test", group = "Test")
public class LocalizationFusionTest extends LinearOpMode {

    private Follower follower;
    private FusionLocalizer fusionLocalizer;
    private ApriltagVision apriltagVision;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Pedro follower using your custom fusion builder method
        follower = Constants.createFusionFollower(hardwareMap);

        // Retrieve the initialized instances directly from your Constants class
        fusionLocalizer = Constants.getFusionLocalizer();
        apriltagVision = Constants.getApriltagVision();

        // Set a baseline zero-pose tracking starting point
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized. Camera starting up...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Standard robot-centric TeleOp drive inputs to manually maneuver the robot
            // Mapping: Left Stick Y = Forward/Backward, Left Stick X = Strafe, Right Stick X = Turn
            follower.update();



            // 1. Update the Pedro follower state machine (this triggers your fusionLocalizer.update())

            // 2. Update the vision pipeline right after the localizer update
            // Passing 'telemetry' allows ApriltagVision to dump its internal tag metrics to the screen
            if (apriltagVision != null && fusionLocalizer != null) {
                apriltagVision.update(fusionLocalizer, telemetry);
            }

            // 3. Grab the comparative telemetry coordinates
            Pose fusedPose = fusionLocalizer.getPose();
            Pose rawOdoPose = Constants.twoWheelLocalizer.getPose();

            // --- Standard Telemetry Logging Layout ---
            telemetry.addLine("=== KALMAN FUSED STATE ===");
            telemetry.addData("Fused X", "%.2f inches", fusedPose.getX());
            telemetry.addData("Fused Y", "%.2f inches", fusedPose.getY());
            telemetry.addData("Fused Heading", "%.2f deg", Math.toDegrees(fusedPose.getHeading()));

            telemetry.addLine("\n=== DEAD RECKONING ONLY ===");
            telemetry.addData("Raw Odo X", "%.2f inches", rawOdoPose.getX());
            telemetry.addData("Raw Odo Y", "%.2f inches", rawOdoPose.getY());
            telemetry.addData("Raw Odo Heading", "%.2f deg", Math.toDegrees(rawOdoPose.getHeading()));

            telemetry.addLine("\n=== FILTER INTEGRITY ===");
            telemetry.addData("Is NaN Detected?", fusionLocalizer.isNAN() ? "YES (Check math/matrices!)" : "No");

            telemetry.update();
        }

        // Safeguard: cleanly close resources down when exiting the OpMode loop
        if (apriltagVision != null) {
            apriltagVision.close();
        }
    }
}