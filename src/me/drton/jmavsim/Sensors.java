package me.drton.jmavsim;

import javax.vecmath.Vector3d;

import me.drton.jmavlib.geo.LatLonAlt;

/**
 * User: ton Date: 26.11.13 Time: 13:32
 */
public interface Sensors {
    void setObject(DynamicObject object);

    Vector3d getAcc();

    Vector3d getGyro();

    Vector3d getMag();

    double getPressureAlt();

    double getPressure();

    double getSonarDist();

    GNSSReport getGNSS();

    LatLonAlt getGlobalPosition();

    boolean isGPSUpdated();

    FlowData getFlowData();

    boolean isFlowUpdated();

    boolean isReset();

    void setReset(boolean reset);

    void setGPSStartTime(long time);

    long getGPSStartTime();

    void update(long t);

    void setParameter(String name, float value);

    float param(String name);

}
