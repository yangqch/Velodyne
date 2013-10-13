package VelodyneDataIO;

public class Point3D {
	public double x;
	public double y;
	public double z;
	
	public Point3D(double x, double y, double z) {
		this.x=x;
		this.y=y;
		this.z=z;
	}
	
	public Point3D minus(Point3D p){
		return new Point3D(this.x-p.x, this.y-p.y, this.z-p.z);
	}
	
	public double calcDist(Point3D p){
		return Math.sqrt((this.x-p.x)*(this.x-p.x) + (this.y-p.y)*(this.y-p.y) + (this.z-p.z)*(this.z-p.z));
	}
	
	public double calcXYDist(Point3D p){
		return Math.sqrt((this.x-p.x)*(this.x-p.x) + (this.y-p.y)*(this.y-p.y) );
	}

	@Override
	public String toString() {
		return String.format("%.1f,%.1f,%.1f", x,y,z);
	}
}

