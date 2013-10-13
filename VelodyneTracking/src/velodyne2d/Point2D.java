package velodyne2d;

public class Point2D {
	public double x, y;
	
	public Point2D(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Point2D minus(Point2D other){
		return new Point2D(x-other.x, y-other.y);
	}
	
	public double distance(Point2D other){
		return Math.sqrt((x-other.x)*(x-other.x) + (y-other.y)*(y-other.y));
	}
	
	public String toString(){
		return String.format("(%.1f,%.1f)", x, y);
	}
}
