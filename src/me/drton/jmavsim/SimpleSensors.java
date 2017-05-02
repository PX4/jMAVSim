package me.drton.jmavsim;

import me.drton.jmavlib.geo.GlobalPositionProjector;
import me.drton.jmavlib.geo.LatLonAlt;
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
    private long gpsStartTime = -1;
    private long gpsInterval = 200;  // [ms]
    private long gpsNext = 0;
    private GNSSReport gps = new GNSSReport();
    private LatLonAlt globalPosition = new LatLonAlt(0, 0, 0);
    private boolean gpsUpdated = false;
    private FlowData flowData = null;
    private long flowInterval = 50;
    private long flowLast = 0;
    private boolean flowUpdated = false;
    //private Matrix3d last_rotation

    private boolean reset = false;
    private double pressureAltOffset = 0.0;
    // default sensor output noise levels
    private float noise_Acc = 0.05f;
    private float noise_Gyo = 0.01f;
    private float noise_Mag = 0.005f;
    private float noise_Prs = 0.01f;
    private float noise_Flw = 0.05f;
    private double magScale = 2.0;   // scaling factor for mag sensor
    private float ephHigh = 100.0f;  // starting GPS horizontal estimation accuracy
    private float ephLow = 0.3f;     // final GPS horizontal estimation accuracy
    private float epvHigh = 100.0f;  // starting GPS vertical estimation accuracy
    private float epvLow = 0.4f;     // final GPS vertical estimation accuracy
    private float fix3Deph = 3.0f;   // maximum h-acc for a "3D" fix
    private float fix2Deph = 4.0f;   // maximum h-acc for a "2D" fix
    // gps noise
    private float gpsNoiseStdDev = 10.f;
    private double randomWalkGpsX = 0.0;
    private double randomWalkGpsY = 0.0;
    private double randomWalkGpsZ = 0.0;
    private double gpsCorrelationTime = 30.0;
    private long prevUpdateTime = 0;
    // accuracy smoothing filters, slowly improve h/v accuracy after startup
    private Filter ephFilter = new Filter();
    private Filter epvFilter = new Filter();

    public SimpleSensors() {
        initFilters();
    }

    private void initFilters() {
        ephFilter.filterInit(1.0 / gpsInterval, 0.9, ephHigh);
        epvFilter.filterInit(1.0 / gpsInterval, 0.9, epvHigh);
    }

    @Override
    public void setObject(DynamicObject object) {
        this.object = object;
        globalProjector.init(object.getWorld().getGlobalReference());
        setGlobalPosition(null);
    }

    public void setGPSStartTime(long time) {
        gpsStartTime = time;
    }

    public long getGPSStartTime() {
        return gpsStartTime;
    }

    public void setGPSDelay(long delay) {
        gpsDelayLine.setDelay(delay);
    }

    public void setGPSInterval(long gpsInterval) {
        this.gpsInterval = gpsInterval;
        // re-init filters with new dt
        initFilters();
    }

    public void setPressureAltOffset(double pressureAltOffset) {
        this.pressureAltOffset = pressureAltOffset;
    }

    public void setNoise_Acc(float noise_Acc) {
        this.noise_Acc = noise_Acc;
    }

    public void setNoise_Gyo(float noise_Gyo) {
        this.noise_Gyo = noise_Gyo;
    }

    public void setNoise_Mag(float noise_Mag) {
        this.noise_Mag = noise_Mag;
    }

    public void setNoise_Prs(float noise_Prs) {
        this.noise_Prs = noise_Prs;
    }

    public void setNoise_Flw(float noise_Flw) {
        this.noise_Flw = noise_Flw;
    }

    public double getMagScale() {
        return magScale;
    }

    public void setMagScale(double magScale) {
        this.magScale = magScale;
    }

    public boolean isReset() {
        return reset;
    }

    public void setReset(boolean reset) {
        this.reset = reset;
    }

    @Override
    public Vector3d getAcc() {
        Vector3d accBody = new Vector3d(object.getAcceleration());
        accBody.sub(object.getWorld().getEnvironment().getG());
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(accBody);
        accBody = addZeroMeanNoise(accBody, noise_Acc);
        return accBody;
    }

    @Override
    public Vector3d getGyro() {
        return addZeroMeanNoise(object.getRotationRate(), noise_Gyo);
    }

    @Override
    public Vector3d getMag() {
        Vector3d mag = new Vector3d(object.getWorld().getEnvironment().getMagField(object.getPosition()));
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(mag);
        mag.scale(magScale);
        return addZeroMeanNoise(mag, noise_Mag);
    }

    @Override
    public double getPressureAlt() {
        return getGlobalPosition().alt + randomNoise(noise_Prs) + pressureAltOffset;
    }

    @Override
    public double getPressure() {
        return SimpleEnvironment.alt2baro(getPressureAlt());
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
    public LatLonAlt getGlobalPosition() {
        return globalPosition;
    }

    public void setGlobalPosition(Vector3d pos) {
        if (pos == null) {
            pos = object.getPosition();
        }

        long t = System.currentTimeMillis();
        double dt = 0.0;
        if (prevUpdateTime > 0) {
            dt = (t - this.prevUpdateTime) * 1e-3;
        }

        // add noise (random walk)
        if (dt > 0.0) {
            double sqrtDt = java.lang.Math.sqrt(dt);
            double noiseX = sqrtDt * randomNoise(gpsNoiseStdDev);
            double noiseY = sqrtDt * randomNoise(gpsNoiseStdDev);
            double noiseZ = sqrtDt * randomNoise(gpsNoiseStdDev);

            this.randomWalkGpsX += noiseX * dt - this.randomWalkGpsX / gpsCorrelationTime;
            this.randomWalkGpsY += noiseY * dt - this.randomWalkGpsY / gpsCorrelationTime;
            this.randomWalkGpsZ += noiseZ * dt - this.randomWalkGpsZ / gpsCorrelationTime;
        }

        double noiseGpsX = pos.x + this.randomWalkGpsX;
        double noiseGpsY = pos.y + this.randomWalkGpsY;
        double noiseGpsZ = pos.z + this.randomWalkGpsZ;

        globalPosition = globalProjector.reproject(new double[] {noiseGpsX, noiseGpsY, noiseGpsZ});

        this.prevUpdateTime = t;
    }

    @Override
    public boolean isGPSUpdated() {
        boolean res = gpsUpdated;
        gpsUpdated = false;
        return res;
    }

    @Override
    public boolean isFlowUpdated() {
        boolean res = flowUpdated;
        flowUpdated = false;
        return res;
    }

    @Override
    public FlowData getFlowData() {
        return flowData;
    }

    @Override
    public void update(long t) {
        float eph, epv;
        setGlobalPosition(null);

        // GPS
        if (gpsStartTime > -1 && t > gpsStartTime && gpsNext <= t) {
            gpsNext = t + gpsInterval;
            gpsUpdated = true;
            GNSSReport gpsCurrent = new GNSSReport();
            eph = (float)ephFilter.filter(ephLow);
            epv = (float)epvFilter.filter(epvLow);

            gpsCurrent.position = globalPosition;
            gpsCurrent.eph = eph;
            gpsCurrent.epv = epv;
            gpsCurrent.velocity = new Vector3d(object.getVelocity());
            gpsCurrent.fix = eph <= fix3Deph ? 3 : eph <= fix2Deph ? 2 : 0;
            gpsCurrent.time = System.currentTimeMillis() * 1000;
            gps = gpsDelayLine.getOutput(t, gpsCurrent);
        }

        if (t > flowLast + flowInterval) {
            flowUpdated = true;
            flowData = new FlowData();

            flowData.integration_time = (t - flowLast) * 1000;
            flowLast = t;

            flowData.distance = getSonarDist();
            double ground_distance = getGroundDistance();

            // Translational flow
            Matrix3d rot = new Matrix3d(object.getRotation());
            Vector3d flow3d = new Vector3d(object.getVelocity());
            flow3d.scale(- 1 / ground_distance);
            rot.transpose();
            rot.transform(flow3d);

            // Rotational flow
            flow3d.x += object.getRotationRate().x;
            flow3d.y += object.getRotationRate().y;

            // Integrated flow
            flow3d = addZeroMeanNoise(flow3d, noise_Flw);
            flowData.integrated_flow = new Vector2d(flow3d.x, flow3d.y);
            flowData.integrated_flow.scale(1e-6 * flowData.integration_time);

            // Integrated gyro
            flowData.integrated_gyro = new Vector3d(object.getRotationRate());
            flowData.integrated_gyro = addZeroMeanNoise(flowData.integrated_gyro, noise_Gyo);
            flowData.integrated_gyro.scale(1e-6 * flowData.integration_time);

            if (ground_distance < 0.3 || ground_distance > 4.0)
            {
                flowData.quality = 200;
            } else {
                flowData.quality = 255;
            }
        }
        else
            flowUpdated = false;
    }

    @Override
    public void setParameter(String name, float value) {
        if ( name.equals("noise_Acc") ) {
            noise_Acc = value;
        }
        else if ( name.equals("noise_Gyo") ) {
            noise_Gyo = value;
        }
        else if ( name.equals("noise_Mag") ) {
            noise_Mag = value;
        }
        else if ( name.equals("noise_Prs") ) {
            noise_Prs = value;
        }
        else if ( name.equals("gpsNoiseStdDev") ) {
            gpsNoiseStdDev = value;
        }
        else if ( name.equals("mass")) {
            object.setMass((double)value);
        }
        else {
            System.out.printf("ERROR: unknown param");
        }
    }

    @Override
    public float param(String name) {
        if ( name.equals("noise_Acc") ) {
            return noise_Acc;
        }
        else if ( name.equals("noise_Gyo") ) {
            return noise_Gyo;
        }
        else if ( name.equals("noise_Mag") ) {
            return noise_Mag;
        }
        else if ( name.equals("noise_Prs") ) {
            return noise_Prs;
        }
        else if ( name.equals("gpsNoiseStdDev") ) {
            return gpsNoiseStdDev;
        }
        else if ( name.equals("mass") ) {
            return (float)object.getMass();
        }
        else {
            System.out.printf("ERROR: unknown param");
        }

        return 0.0f;
    }

    // Utility methods

    public double randomNoise(float stdDev) {
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

        return x0 * stdDev;
    }

    public Vector3d addZeroMeanNoise(Vector3d vIn, float stdDev) {

        return new Vector3d(vIn.x + randomNoise(stdDev),
                            vIn.y + randomNoise(stdDev),
                            vIn.z + randomNoise(stdDev));
    }

}
