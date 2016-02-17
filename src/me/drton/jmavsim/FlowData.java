package me.drton.jmavsim;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * Created by v01d on 15/09/15.
 */
public class FlowData {
    public long integration_time;
    public Vector2d integrated_flow = new Vector2d();
    public Vector3d integrated_gyro = new Vector3d();
    public double distance;
    public int quality;
}
