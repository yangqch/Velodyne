package VelodyneView;

public class CameraControl{
	static double DEG2RAD=Math.PI/180.0;
	static double RAD2DEG=180.0/Math.PI;
	
	public float rotAngle;//right-hand from negative x-axis 
	public float vertAngle;//up: negative z-axis
	public float dist;//distance to origin
	
	public float x,y,z;//camera position w.r.t world frame
	public float lookat_x,lookat_y,lookat_z;//camera position w.r.t world frame
	
	public CameraControl(float rot, float vert, float dist) {
		this.rotAngle=rot;
		this.vertAngle=vert;
		this.dist=dist;
		this.update();
		lookat_x=0;
		lookat_y=0;
		lookat_z=0;
	}
	
	public void update(){
		z = this.dist * (float)Math.sin(this.vertAngle*DEG2RAD);
		float xyDist = this.dist * (float)Math.cos(this.vertAngle*DEG2RAD);
		x = xyDist * (float)Math.cos(this.rotAngle*DEG2RAD);
		y = xyDist * (float)Math.sin(this.rotAngle*DEG2RAD);		
	}
	
	public String toString(){
		return String.format("cam pos %.1f, %.1f, %.1f looking at %.1f, %.1f, %.1f\n",x,y,z, lookat_x, lookat_y, lookat_z);
	}
}