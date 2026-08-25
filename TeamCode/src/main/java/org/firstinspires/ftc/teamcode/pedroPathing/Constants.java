package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("front_left");
                c.backLeftName.set("back_left");
                c.frontRightName.set("front_right");
                c.backRightName.set("back_right");
                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                //c.xPodOffset.set(-6.74818);
                //c.yPodOffset.set(1.33282);
                c.xPodOffset.set(-9.49301136);
                c.yPodOffset.set(2.721003331);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            }
    );

    public static TwoWheelConfig noPPlocalizerConfig = new TwoWheelConfig(
            c -> {
                c.xPodName.set("leftFront");
                c.yPodName.set("rightRear");
                c.imuName.set("imu");
                c.imuOrientation.set(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
                c.xPodDirection.set(Encoder.FORWARD);
                c.yPodDirection.set(Encoder.REVERSE);
                c.forwardTicksToInches.set(0.1);
                c.strafeTicksToInches.set(0.2);
            }
    );
public static OTOSConfig otosConfig = new OTOSConfig(
        c -> {
            c.name.set("otos");
            c.linearUnit.set(DistanceUnit.INCH);
            c.offset.set(new Pose(3,5));
            c.linearScalar.set(0.5);
            c.angularScalar.set(0.5);
        }
);
public static ThreeWheelConfig threeWheelConfig = new ThreeWheelConfig(
        c -> {
            c.forwardTicksToInches.set(0.5);
            c.strafeTicksToInches.set(0.1);
            c.turnTicksToRadians.set(0.5);
            c.leftPodY.set(0.5);
            c.rightPodY.set(0.3);
            c.strafePodX.set(0.2);
            c.leftEncoderName.set("leftE");
            c.rightEncoderName.set("rightE");
            c.strafeEncoderName.set("strafE");
            c.leftEncoderDirection.set(Encoder.FORWARD);

        }
);
public static ThreeWheelIMUConfig threeWheelIMUConfig = new ThreeWheelIMUConfig(
        c -> {
            c.forwardTicksToInches.set(0.5);
            c.strafeTicksToInches.set(0.1);
            c.turnTicksToRadians.set(0.5);
            c.leftPodY.set(0.5);
            c.rightPodY.set(0.3);
            c.strafePodX.set(0.2);
            c.leftEncoderName.set("leftE");
            c.rightEncoderName.set("rightE");
            c.strafeEncoderName.set("strafE");
            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.imuName.set("imu");
            c.imuOrientation.set (new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        }
);
public static OctoQuadConfig octoQuadConfig = new OctoQuadConfig(
        c -> {
            c.name.set("octoquad");
            c.ticksPerUnit.set(19.89436789);
            c.encoderResolutionUnit.set(DistanceUnit.MM);
            c.headingScalar.set(1.0168);
            c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.xPodOffset.set(-3.95);
            c.yPodOffset.set(-5.67);
        }
);
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.brakeController.set(Controller.pid(0.2,0,0));
                c.headingController.set(Controller.piecewise(
                        Controller.pid(1.2901, 0, 0.1731)
                ).put(
                        Math.PI / 20,
                        Controller.pid(2.9028, 0, 0.3304)
                ));
                c.headingFeedforward.set(Controller.dynamicFeedforward(0.1416));
                Controller largeTranslationalForward = Controller.pid(0.019,0,0).plus(Controller.staticFeedforward(0.0119));
                Controller smallTranslationalForward = Controller.pid(0.0126,0,0);
                Controller smallTranslationalLateral = Controller.pid(0.0381,0,0).plus(Controller.staticFeedforward(0.0005));
                Controller largeTranslationalLateral = Controller.pid(0.0266,0,0).plus(Controller.staticFeedforward(0.01));
                c.forwardTranslationalController.set(Controller.piecewise(
                        Controller.staticFeedforward(0)
                ).put(0.5, smallTranslationalForward).put(2.5, largeTranslationalForward));
                c.lateralTranslationalController.set(Controller.piecewise(Controller.staticFeedforward(0)).put(0.5, smallTranslationalLateral).put(2.5, largeTranslationalLateral));
                c.linearBrakeCoefficients.set(Matrix.diag(0.09862195851130677, 0.09862195851130677));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018122004959482967, 0.0018122004959482967));
                c.maxAchievableForwardVelocity.set(82.98255);
                c.maxAchievableStrafeVelocity.set(66.6340731);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new TwoWheelLocalizer(h, noPPlocalizerConfig), new Mecanum(h, driveConfig), new Foresight(foresightConfig));
    }
}