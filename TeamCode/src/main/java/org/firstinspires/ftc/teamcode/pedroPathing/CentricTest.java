package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CentricTest extends OpMode {
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public boolean useFieldCentric = false;
    public double offsetHeading = Math.toRadians(180);
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        if (useFieldCentric)
            follower.manual(ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading(), offsetHeading));
        else
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.xWasPressed())
            useFieldCentric = !useFieldCentric;

        if (gamepad1.aWasPressed())
            offsetHeading += Math.toRadians(180);

        follower.update();
        multipleTelemetry.addData("Loop Time Hz", 1000/loopTime);
        multipleTelemetry.addData("Mode", follower.mode());
        multipleTelemetry.addData("Manual?", follower.manual());
        multipleTelemetry.addData("Pose", follower.pose());
        multipleTelemetry.update();
    }
}
