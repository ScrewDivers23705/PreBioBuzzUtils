package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ApriltagVision;

import java.util.HashMap;
import java.util.HashSet;

public class Constants {

    public static TwoWheelLocalizer twoWheelLocalizer;
    public static FusionLocalizer fusionLocalizer;
    public static ApriltagVision apriltagVision;

    public static FusionLocalizer getFusionLocalizer() { return fusionLocalizer; }
    public static ApriltagVision getApriltagVision() { return apriltagVision; }

    public static Follower createFusionFollower(HardwareMap hardwareMap) {
        twoWheelLocalizer = new TwoWheelLocalizer(hardwareMap, localizerConstants);

        fusionLocalizer = new FusionLocalizer(
            twoWheelLocalizer,
            new Pose(1.0,1.0,Math.toRadians(5)), //Initial unceartenty
            new Pose(0.5,0.5,Math.toRadians(1)), // Process noise
            new Pose(0.5,0.5,Math.toRadians(3)), // deafult vision variance
            100
        );
        HashSet<Integer> ids = new HashSet<>();
        ids.add(20);
        ids.add(21);
        ids.add(22);
        apriltagVision = new ApriltagVision(
                hardwareMap, "Webcam 1",
                new Position(DistanceUnit.INCH,-2,13,0,0),
                new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0),
                822,822,320,240,
                ids
        );

        return new FollowerBuilder(followerConstants,hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(fusionLocalizer)
                .build();
    }


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .centripetalScaling(0)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9,0.0005,0.019,0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.085, 0.00003, 0.0002, 0.003))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016)); // (kP, kLinear, kQuadratic);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("right_front")
            .rightRearMotorName("right_back")
            .leftRearMotorName("left_back")
            .leftFrontMotorName("left_front")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.5683647148818)
            .yVelocity(54.91491995416327);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("forward_odo")
            .strafeEncoder_HardwareMapName("left_back")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardPodY(3.212783150010722)
            .strafePodX( -6.938853076135274)
            .forwardTicksToInches(0.002967751849349) //DONE check up the multiplier by running the localization opmode
            .strafeTicksToInches(0.002946318869670);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}