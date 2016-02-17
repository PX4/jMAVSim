package me.drton.jmavsim;

import me.drton.jmavlib.geo.GlobalPositionProjector;
import me.drton.jmavlib.processing.DelayLine;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * User: ton Date: 27.11.13 Time: 19:06
 */
public class SimpleSensors implements Sensors {
    private DynamicObject object;
    private GlobalPositionProjector globalProjector = new GlobalPositionProjector();
    private DelayLine<GNSSReport> gpsDelayLine = new DelayLine<GNSSReport>();
    private long gpsStartTime = 0;
    private long gpsInterval = 200;
    private long gpsLast = 0;
    private GNSSReport gps = new GNSSReport();
    private boolean gpsUpdated = false;
    private double pressureAltOffset = 0.0;

    private FlowData flowData = null;
    private long flowInterval = 50;
    private long flowLast = 0;
    private boolean flowUpdated = false;
    //private Matrix3d last_rotation

    @Override
    public void setObject(DynamicObject object) {
        this.object = object;
        globalProjector.init(object.getWorld().getGlobalReference());
    }

    public double randomNoise(double stdDev) {
        double x0;
        double b0, b1;

        do {
            b0 = Math.random();
            b1 = Math.random();
        } while (b0 <= Float.intBitsToFloat(0x1));

        x0 = java.lang.Math.sqrt(-2.0 * java.lang.Math.log(b0)) * java.lang.Math.cos(Math.PI * 2.0 * b1);

        if (java.lang.Double.isInfinite(x0) || java.lang.Double.isNaN(x0)) {
            x0 = 0.0;
        }

        return x0 * (stdDev * stdDev);
    }

    public Vector3d addZeroMeanNoise(Vector3d vIn, double stdDev) {

        return new Vector3d(vIn.x + randomNoise(stdDev),
                            vIn.y + randomNoise(stdDev),
                            vIn.z + randomNoise(stdDev));
    }

    public void setGPSStartTime(long time) {
        gpsStartTime = time;
    }

    public void setGPSDelay(long delay) {
        gpsDelayLine.setDelay(delay);
    }

    public void setGPSInterval(long gpsInterval) {
        this.gpsInterval = gpsInterval;
    }

    public void setPressureAltOffset(double pressureAltOffset) {
        this.pressureAltOffset = pressureAltOffset;
    }

    @Override
    public Vector3d getAcc() {
        Vector3d accBody = new Vector3d(object.getAcceleration());
        accBody.sub(object.getWorld().getEnvironment().getG());
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(accBody);
        accBody = addZeroMeanNoise(accBody, 0.05);
        return accBody;
    }

    @Override
    public Vector3d getGyro() {
        return addZeroMeanNoise(object.getRotationRate(), 0.01);
    }

    @Override
    public Vector3d getMag() {
        Vector3d mag = new Vector3d(object.getWorld().getEnvironment().getMagField(object.getPosition()));
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(mag);
        return addZeroMeanNoise(mag, 0.005);
    }

    @Override
    public double getPressureAlt() {
        return -object.getPosition().z + pressureAltOffset;
    }

    @Override
    public double getSonarDist() {
        double dist = getGroundDistance();
        if (dist < 0.3) dist = 0.3;
        else if (dist > 4.0) dist = 4.0;

        return dist;
    }

    private double getGroundDistance()
    {
        double height = -object.getPosition().z;
        Matrix3d rot = new Matrix3d(object.getRotation());
        return height / rot.getM22();
    }

    @Override
    public GNSSReport getGNSS() {
        return gps;
    }

    @Override
    public boolean isGPSUpdated() {
        boolean res = gpsUpdated;
        gpsUpdated = false;
        return res;
    }

    @Override
    public FlowData getFlowData() {
        if (flowUpdated)
            return flowData;
        else
            return null;
    }

    @Override
    public void update(long t) {
        // GPS
        if (t > gpsStartTime && t > gpsLast + gpsInterval) {
            gpsLast = t;
            gpsUpdated = true;
            GNSSReport gpsCurrent = new GNSSReport();
            Vector3d pos = object.getPosition();
            gpsCurrent.position = globalProjector.reproject(new double[]{pos.x, pos.y, pos.z});
            gpsCurrent.eph = 1.0;
            gpsCurrent.epv = 1.0;
            gpsCurrent.velocity = new Vector3d(object.getVelocity());
            gpsCurrent.fix = 3;
            gpsCurrent.time = System.currentTimeMillis() * 1000;
            gps = gpsDelayLine.getOutput(t, gpsCurrent);
        }

        if (t > flowLast + flowInterval) {
            flowUpdated = true;
            if (flowData == null) { flowData = new FlowData(); return; }

            flowData.integration_time = (flowLast - t) * 1000;
            flowLast = t;
            flowData.distance = getSonarDist();

            double ground_distance = getGroundDistance();
            if (ground_distance < 0.3 || ground_distance > 4.0) // TODO: could also depend on angle and velocity
            {
                flowData.quality = 0;
            } else {
                //Vector2d xy_velocity = new Vector2d(object.getVelocity().x, object.getVelocity().y);
                //flowData.integrated_flow = xy_velocity.scale((1/ground_distance) * flowData.integration_time); // TODO: needs adding rotational flow
                //flowData.integrated_gyro = ; TODO: compute this from delta orientation between updates
                //flowData.quality = 255; // TODO: to be enabled once this is finished
                flowData.quality = 0;
            }
        }
        else
            flowUpdated = false;
    }
}
