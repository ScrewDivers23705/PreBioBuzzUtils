package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.SimplePath;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LineTest extends OpMode {
    public static double DISTANCE = 40;

    private Path line;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.pose(new Pose(72, 72, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        Curve forwards = new Line(new Pose(72,72, 0), new Pose(DISTANCE + 72,72, 0));
        Curve backwards = new Line(new Pose(DISTANCE + 72,72, 0), new Pose(72,72, 0));
        line = new CompoundPath(new SimplePath(forwards, Interpolator.constant(0)), new SimplePath(backwards, Interpolator.constant(0)));
        follower.follow(line);
    }

    @Override
    public void loop() {
        follower.update();
        multipleTelemetry.addData("Is Busy?", follower.isBusy());
        multipleTelemetry.addData("Pose", follower.pose());
        multipleTelemetry.update();
    }
}
