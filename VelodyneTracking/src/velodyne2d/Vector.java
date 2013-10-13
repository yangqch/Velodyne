package velodyne2d;

public class Vector {
	public double x;
	public double y;
	
	public Vector(double x, double y) {
		this.x=x;
		this.y=y;
	}
	
	public Vector(Point2D p1, Point2D p2){
		this.x = p2.x - p1.x;
		this.y = p2.y - p1.y;
	}
	
	public Vector(double rad){
		this.x = Math.cos(rad);
		this.y = Math.sin(rad);
	}
	
	public double dot(Vector vec){
		return this.x*vec.x + this.y*vec.y;
	}
	
	public void normalize(){
		double n = this.norm();
		if(n==0) return;
		this.x /= n;
		this.y /= n;
	}
	
	public double norm(){
		return Math.sqrt(x*x + y*y);
	}
	
	public double normSq(){
		return x*x + y*y;
	}
	
	public Vector makeNormalVector(){
		double n = this.norm();
		return new Vector(-y/n, x/n);
	}
	
	/**
	 * make normal vector from p1 to p2
	 * @param p1
	 * @param p2
	 * @return
	 */
	public static Vector makeNormalVector(Point2D p1, Point2D p2){
		double dist = Math.sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
		return new Vector( (p2.x-p1.x)/dist, (p2.y-p1.y)/dist );
	}
	
	public String toString(){
		return String.format("(%.2f,%.2f)", x, y);
	}
}
