package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // Kept from your own previously-tuned values, not the reference's - PIDF/mass
    // gains are specific to your robot's weight and drivetrain feel. Copying another
    // team's gains here would likely make your heading control oscillate or lag,
    // since it was tuned against a physically different robot.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .centripetalScaling(0)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0005, 0.019, 0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.085, 0.00003, 0.0002, 0.003))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016));

    // Motor NAMES renamed to frontLeft/frontRight/backLeft/backRight to match what
    // Drivetrain.java's context.motor(...) calls expect - you'll need to rename these
    // 4 motors in your driver station hardware configuration to match (or, if you'd
    // rather not touch the config, change Drivetrain.java's context.motor(...) calls
    // to your actual existing names instead - just keep this file and that file
    // pointing at the same 4 physical motors). Directions and velocities below are
    // your own real measured values, just remapped onto the new names 1:1.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("left_front")
            .leftRearMotorName("left_back")
            .rightFrontMotorName("right_front")
            .rightRearMotorName("right_back")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.5683647148818)
            .yVelocity(54.91491995416327);

    // Your actual odometry hardware: standard two-wheel dead wheels read through the
    // Control Hub, plus the Control Hub's own built-in IMU - no Pinpoint odometry
    // computer. This replaces the reference's PinpointConstants entirely.
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
            .strafePodX(-6.938853076135274)
            .forwardTicksToInches(0.002967751849349)
            .strafeTicksToInches(0.002946318869670);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Plain odometry follower with no vision fusion at all - useful for isolated
     * encoder-tick or PIDF tuning OpModes where you don't want vision in the loop.
     * Your Drivetrain subsystem builds its own fusion-enabled Follower directly and
     * does not use this method.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}