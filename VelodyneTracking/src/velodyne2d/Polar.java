package velodyne2d;

public class Polar{
	public double angle;
	public double dist;
	public Polar(double a, double d) {
		angle = a;
		dist = d;
	}
	public double getRadius(){
		return angle*VirtualScan.DEG2RAD;
	}
}