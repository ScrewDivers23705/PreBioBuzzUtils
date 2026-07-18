package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * credit MOE365 the GOATS
 * PREDICT (every update()):
 *   Instead of integrating a velocity estimate, this takes the RAW POSE DELTA
 *   between consecutive dead-reckoning readings (as a proper SE(2) relative
 *   transform) and composes it onto the fused pose. This trusts the
 *   localizer's own already-integrated position tracking directly, rather
 *   than re-deriving position from what may be a noisier velocity estimate.
 *
 *   Uncertainty (P) grows by process noise scaled by DISTANCE ACTUALLY
 *   TRAVELED in the robot's own frame (not just elapsed time), then rotated
 *   into world frame. A robot sitting still shouldn't accrue position
 *   uncertainty just because the clock ticked; a robot moving fast should
 *   accrue it faster (more chances to slip).
 *
 * UPDATE (addMeasurement(), called whenever a vision reading is ready):
 *   Vision measurements usually describe where the robot WAS a frame or two
 *   ago (camera + pipeline latency). The correction is applied at its true
 *   capture timestamp against a buffered pose history, then every subsequent
 *   step is replayed forward from the corrected point - using exponential-map
 *   interpolation so replaying across a curved path segment doesn't cut
 *   corners the way straight-line interpolation would.
 */
public class FusionLocalizer implements Localizer {

    private static class KalmanState {
        Pose pose;
        Pose twist;
        Pose relativeTransform;
        Matrix covariance;

        KalmanState(Pose pose, Pose twist, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.twist = twist;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    /** Floor for covariance diagonal entries, so floating-point error can't drive variance to zero/negative. */
    public static double EPSILON = 1e-6;

    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose currentRelativeTransform;
    private Matrix P; // state covariance
    private final Matrix Q; // process noise density, per unit distance traveled
    private final Matrix R; // default measurement noise covariance

    private long lastUpdateTime = -1;
    private final int bufferSize;
    private final NavigableMap<Long, KalmanState> history = new TreeMap<>();

    /**
     * @param deadReckoning      existing odometry Localizer
     * @param initialVariance    starting variance per axis
     * @param processVariance    how much distrust you put in odometry's own consistency per unit distance traveled
     * @param measurementVariance default vision measurement variance, used unless addMeasurement gets a per-call override
     * @param bufferSize         how many recent samples to retain for replaying delayed vision corrections -
     *                           size to cover your worst-case vision latency at your loop rate
     */
    public FusionLocalizer(Localizer deadReckoning, Pose initialVariance, Pose processVariance, Pose measurementVariance, int bufferSize) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();
        this.currentRawPose = new Pose();

        this.P = Matrix.diag(
                Math.max(initialVariance.getX(), EPSILON),
                Math.max(initialVariance.getY(), EPSILON),
                Math.max(initialVariance.getHeading(), EPSILON));
        this.Q = Matrix.diag(
                Math.max(processVariance.getX(), EPSILON),
                Math.max(processVariance.getY(), EPSILON),
                Math.max(processVariance.getHeading(), EPSILON));
        this.R = Matrix.diag(
                Math.max(measurementVariance.getX(), EPSILON),
                Math.max(measurementVariance.getY(), EPSILON),
                Math.max(measurementVariance.getHeading(), EPSILON));
        this.bufferSize = bufferSize;

        history.put(0L, new KalmanState(currentPosition, new Pose(), currentRawPose, P));
    }

    @Override
    public void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        currentVelocity = deadReckoning.getVelocity();

        Pose rawPose = deadReckoning.getPose();
        currentRelativeTransform = compose(invert(currentRawPose), rawPose);

        P = growUncertainty(P, currentPosition, currentVelocity, dt);
        currentPosition = compose(currentPosition, currentRelativeTransform);
        currentRawPose = rawPose;

        history.put(now, new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P));
        if (history.size() > bufferSize) history.pollFirstEntry();
    }

    /**
     * Grows covariance by process noise scaled by distance actually traveled in the
     * robot's own frame this step, then rotates that into world frame so x/y
     * uncertainty become correlated once the robot has turned.
     */
    private Matrix growUncertainty(Matrix P, Pose pose, Pose twist, double dt) {
        if (twist == null) {
            twist = new Pose(0, 0, 0);
        }
        // Rotate the world-frame twist into the robot's own frame manually (avoids
        // depending on a Pose.rotate() overload we haven't independently verified).
        double h = -pose.getHeading();
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double bodyVx = twist.getX() * cos - twist.getY() * sin;
        double bodyVy = twist.getX() * sin + twist.getY() * cos;
        double bodyOmega = twist.getHeading(); // angular velocity is frame-invariant in 2D

        double distX = Math.abs(bodyVx * dt);
        double distY = Math.abs(bodyVy * dt);
        double distTheta = Math.abs(bodyOmega * dt);

        Matrix bodyQ = Matrix.diag(
                distX * Q.get(0, 0),
                distY * Q.get(1, 1),
                distTheta * Q.get(2, 2));

        Matrix rotation = Matrix.createRotation(pose.getHeading());
        Matrix worldQ = rotation.multiply(bodyQ).multiply(rotation.transposed());

        Matrix grown = P.plus(worldQ);
        clampCovariance(grown);
        return grown;
    }

    private KalmanState currentState() {
        return new KalmanState(currentPosition, currentVelocity, currentRelativeTransform, P);
    }

    /** Add a vision measurement using the default measurement noise. */
    public void addMeasurement(Pose measuredPose, long captureTimeNanos) {
        addMeasurement(measuredPose, captureTimeNanos, null);
    }

    /**
     * Add a vision measurement captured at captureTimeNanos (the actual moment the frame
     * was captured, before pipeline processing delay - not "now"). Set any axis of
     * measuredPose to NaN to skip fusing that axis.
     *
     * @param customVariance per-call measurement variance override, or null to use the default R
     */
    public void addMeasurement(Pose measuredPose, long captureTimeNanos, Pose customVariance) {
        Matrix measurementR = customVariance == null
                ? R
                : Matrix.diag(
                Math.max(customVariance.getX(), EPSILON),
                Math.max(customVariance.getY(), EPSILON),
                Math.max(customVariance.getHeading(), EPSILON));

        if (history.isEmpty() || captureTimeNanos < history.firstKey() || captureTimeNanos > history.lastKey()) {
            return; // outside our replay buffer window - can't place it correctly, so skip it
        }

        KalmanState interpolated = interpolate(captureTimeNanos);
        if (interpolated == null) interpolated = currentState();
        Pose pastPose = interpolated.pose;

        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        boolean measH = !Double.isNaN(measuredPose.getHeading());

        Matrix innovation = new Matrix(new double[][]{
                {measX ? measuredPose.getX() - pastPose.getX() : 0},
                {measY ? measuredPose.getY() - pastPose.getY() : 0},
                {measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - pastPose.getHeading()) : 0}
        });
        Matrix mask = Matrix.diag(measX ? 1 : 0, measY ? 1 : 0, measH ? 1 : 0);

        Matrix Pm = interpolated.covariance;
        Matrix S = Pm.plus(measurementR);
        Matrix sInverse = invert(S);
        if (sInverse == null) return; // singular innovation covariance - skip rather than risk garbage output

        Matrix K = mask.multiply(Pm.multiply(sInverse));
        Matrix maskedInnovation = mask.multiply(innovation);
        Matrix correction = K.multiply(maskedInnovation);

        Pose correctedPast = new Pose(
                pastPose.getX() + correction.get(0, 0),
                pastPose.getY() + correction.get(1, 0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + correction.get(2, 0)));

        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix correctedCovariance = IK.multiply(Pm).multiply(IK.transposed())
                .plus(K.multiply(measurementR).multiply(K.transposed()));
        clampCovariance(correctedCovariance);

        history.put(captureTimeNanos, new KalmanState(
                correctedPast, interpolated.twist, interpolated.relativeTransform, correctedCovariance));

        replayForward(captureTimeNanos, correctedPast, correctedCovariance);

        currentPosition = history.lastEntry().getValue().pose;
        P = history.lastEntry().getValue().covariance;
    }

    /** Re-applies every buffered relative transform after the correction point, so the fix propagates to "now". */
    private void replayForward(long fromTimeNanos, Pose fromPose, Matrix fromCovariance) {
        long prevTime = fromTimeNanos;
        Pose prevPose = fromPose;
        Matrix cov = fromCovariance;

        for (NavigableMap.Entry<Long, KalmanState> entry : history.tailMap(fromTimeNanos, false).entrySet()) {
            long t = entry.getKey();
            Pose twist = entry.getValue().twist;
            if (twist == null) twist = getVelocity();

            if (twist == null) twist = new Pose(0, 0, 0);
            double dt = (t - prevTime) / 1e9;
            cov = growUncertainty(cov, prevPose, twist, dt);
            prevPose = compose(prevPose, entry.getValue().relativeTransform);

            history.put(t, new KalmanState(prevPose, twist, entry.getValue().relativeTransform, cov));
            prevTime = t;
        }
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);
        if (lowerKey == null || upperKey == null) return null;

        KalmanState lower = history.get(lowerKey);
        if (lowerKey.equals(upperKey)) return lower;
        KalmanState upper = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);
        Pose pose = lerpPose(lower.pose, upper.pose, ratio);
        Pose twist = lerpPose(lower.twist, upper.twist, ratio);
        Pose relativeTransform = interpolateTransform(lower.relativeTransform, upper.relativeTransform, ratio);

        // Using the earlier sample's covariance is a deliberate simplification - interpolating
        // covariance itself isn't meaningful the same way pose is, so we take the more
        // conservative (typically larger, since P grows over time) of the two endpoints.
        return new KalmanState(pose, twist, relativeTransform, lower.covariance);
    }

    private static Pose lerpPose(Pose a, Pose b, double ratio) {
        double x = a.getX() + ratio * (b.getX() - a.getX());
        double y = a.getY() + ratio * (b.getY() - a.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(b.getHeading(), a.getHeading());
        double heading = MathFunctions.normalizeAngle(a.getHeading() + ratio * headingDiff);
        return new Pose(x, y, heading);
    }

    /**
     * Interpolates a relative SE(2) transform using the exponential map, so replaying
     * across a curved path segment doesn't cut corners the way linearly interpolating
     * x/y independently of heading would. Treats the lerped (dx, dy, dtheta) as a
     * constant-curvature arc and solves for the pose that arc actually reaches.
     */
    private static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        double dx = a.getX() + ratio * (b.getX() - a.getX());
        double dy = a.getY() + ratio * (b.getY() - a.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(b.getHeading(), a.getHeading());
        double dtheta = MathFunctions.normalizeAngle(a.getHeading() + ratio * headingDiff);

        double eps = 1e-4;
        double x, y;
        if (Math.abs(dtheta) < eps) {
            x = dx;
            y = dy;
        } else {
            double sinT = Math.sin(dtheta);
            double cosT = Math.cos(dtheta);
            double v = sinT / dtheta;
            double w = (1 - cosT) / dtheta;
            x = v * dx - w * dy;
            y = w * dx + v * dy;
        }
        return new Pose(x, y, dtheta);
    }

    /** SE(2) composition: apply relative transform b on top of absolute pose a. */
    private static Pose compose(Pose a, Pose b) {
        double cos = Math.cos(a.getHeading());
        double sin = Math.sin(a.getHeading());
        double x = a.getX() + b.getX() * cos - b.getY() * sin;
        double y = a.getY() + b.getX() * sin + b.getY() * cos;
        double h = MathFunctions.normalizeAngle(a.getHeading() + b.getHeading());
        return new Pose(x, y, h);
    }

    /** SE(2) inverse. */
    private static Pose invert(Pose pose) {
        double cos = Math.cos(pose.getHeading());
        double sin = Math.sin(pose.getHeading());
        double x = -pose.getX() * cos - pose.getY() * sin;
        double y = pose.getX() * sin - pose.getY() * cos;
        double h = -pose.getHeading();
        return new Pose(x, y, h);
    }

    private void clampCovariance(Matrix P) {
        for (int i = 0; i < 3; i++) {
            if (P.get(i, i) < EPSILON) P.set(i, i, EPSILON);
        }
    }

    /** Matrix inverse via row reduction, returning null (rather than garbage) on a singular matrix. */
    private static Matrix invert(Matrix matrix) {
        if (matrix.getRows() != matrix.getColumns()) return null;
        Matrix I = Matrix.identity(matrix.getRows());
        Matrix[] reduced = Matrix.rref(matrix, I);
        if (!reduced[0].equals(I)) return null;
        return reduced[1];
    }

    @Override
    public Pose getPose() {
        return currentPosition;
    }

    @Override
    public Pose getVelocity() {
        // 1. Check if our fused velocity exists
        if (currentVelocity != null) {
            return currentVelocity;
        }

        // 2. Fallback to raw odometry velocity
        Pose rawVel = deadReckoning.getVelocity();
        if (rawVel != null) {
            return rawVel;
        }

        // 3. Absolute safety net: if everything is null, return a static zero-velocity pose
        return new Pose(0, 0, 0);
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        history.put(0L, new KalmanState(setStart, new Pose(), setStart, P));
        currentPosition = setStart;
        currentRawPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose;
        deadReckoning.setPose(setPose);
        currentRawPose = setPose;
        if (!history.isEmpty()) {
            history.lastEntry().getValue().pose = setPose;
        } else {
            setStartPose(setPose);
        }
    }

    @Override
    public double getTotalHeading() {
        return currentPosition.getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return deadReckoning.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return deadReckoning.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return deadReckoning.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        deadReckoning.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return deadReckoning.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPosition.getX())
                || Double.isNaN(currentPosition.getY())
                || Double.isNaN(currentPosition.getHeading());
    }
}