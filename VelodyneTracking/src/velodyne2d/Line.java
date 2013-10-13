package velodyne2d;

public class Line {
	static double DEG2RAD=Math.PI/180.0;
	static double RAD2DEG=180.0/Math.PI;
	
	public Point2D p1;
	public Point2D p2;
	public double angle;//(-pi ~ +pi)
	public double length;
	
	public Line(Point2D p1, Point2D p2) {
		this.p1 = p1;
		this.p2 = p2;
		//calculate angle and length
		this.length = Math.sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
		this.angle = Math.atan2(p1.y-p2.y, p1.x-p2.x);
	}
	
	public Line(Line other){
		this.p1 = other.p1;
		this.p2 = other.p2;
		this.angle = other.angle;
		this.length = other.length;
	}
	
	public Line(Point2D[] pts){
		//linear regression for line
		
		//project first and last points to line as p1 and p2
	}
	
	public double getAngleDeg(){
		return this.angle*RAD2DEG;
	}
	
	public double getPerpendicularAngleRad(){
		double theta = this.angle*RAD2DEG + 90; //-90~270
		theta = theta>90 ? theta-180 : theta; //-90 ~ 90
		return theta * DEG2RAD;
	}
	
	public Point2D getMidPoint(){
		return new Point2D((p1.x+p2.x)/2, (p1.y+p2.y)/2);
	}
	
	/**
	 * calculate the distance from p to this line
	 * vector p1p2 is x-axis, y-axis follows right-hand rule
	 * return the projection of p1p on y-axis, positive or negative 
	 * @param p
	 * @return
	 */
	public double calcDist(Point2D p){
		Vector p1p2 = new Vector(this.p1, this.p2);
		Vector normal = p1p2.makeNormalVector();
		Vector p1p = new Vector(this.p1, p);
		return p1p.dot(normal);
	}
	
	public String toString(){
		return String.format("line(%s, %s)", this.p1, this.p2);
	}
}
