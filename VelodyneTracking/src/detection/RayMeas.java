package detection;

import velodyne2d.CoordinateFrame2D;
import velodyne2d.FrameTransformer2D;
import velodyne2d.Point2D;

public class RayMeas {
	public int idx;
	public double distance;
	public double diff;
	public Point2D real;//real measurement in Point2D
	
	public RayMeas(int idx, double d) {
		this.idx = idx;
		this.distance = d;
		this.diff=0;
		this.real = null;
	}
}
