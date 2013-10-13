package calibration;

import java.nio.FloatBuffer;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.ejml.ops.SingularOps;
import org.ejml.simple.SimpleMatrix;

import VelodyneDataIO.Point3D;

import com.jogamp.common.nio.Buffers;

public class GroundPlanePCA {
	//for SVD
	final private int dataDimension = 3; 
	private DenseMatrix64F A = new DenseMatrix64F(1, 1);//data
	private DenseMatrix64F prinComps = new DenseMatrix64F(dataDimension, dataDimension);// = new DenseMatrix64F(dataDimension, dataDimension);//principle components
	private DenseMatrix64F singValue = new DenseMatrix64F(dataDimension, dataDimension);;
	private DenseMatrix64F mean = new DenseMatrix64F(dataDimension, 1);
	
	//data info
	private int sampleNum = 0;
	private int sampleIdx = 0;
	
	//for plane parameter
	//O is the ground frame located at the center of data o
	//proj is the new frame parallel to O, the x and y are projection of L_x, L_y to xy plane(ground plane) of O
	private SimpleMatrix L_R_O = SimpleMatrix.wrap(this.prinComps);//ground plane frame in Lidar system
	
	private SimpleMatrix o = SimpleMatrix.wrap(mean);//the central of data, origin point of plane
	
	private SimpleMatrix O_proj = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, 1));
	private SimpleMatrix L_proj = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, 1));
	
	private SimpleMatrix proj_R_L = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, dataDimension));
	private SimpleMatrix proj_R_O = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, dataDimension));
	
	private SimpleMatrix tVec = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, 1));
	
	//private double[] xgrid, ygrid, zgrid;
	private SimpleMatrix grid;
	//private SimpleMatrix planez_b = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, dataDimension));
	private SimpleMatrix L_xgrid, L_ygrid, L_zgrid;//grid in Lidar frame
	
	public GroundPlanePCA() {
	}
	
	public void addData(List<Point3D> groundPoints){
		this.sampleNum = groundPoints.size();
		A.reshape(dataDimension, sampleNum);
		for(int i=0; i<groundPoints.size(); i++){
			this.A.set(0, sampleIdx, groundPoints.get(i).x);
			this.A.set(1, sampleIdx, groundPoints.get(i).y);
			this.A.set(2, sampleIdx, groundPoints.get(i).z);
			sampleIdx++;
		}
	}
	
	public void computeBasis(){
		//compute mean of each dimension
		double[] sum = new double[this.dataDimension];
		for(int i=0; i<this.dataDimension; i++){
			for(int j=0; j<this.sampleNum; j++){
				sum[i]+=A.get(i, j);
			}
		}
		for(int i=0; i<this.dataDimension; i++){
			mean.set(i, 0, sum[i]/this.sampleNum);
		}
		//subtract mean from A 
		for(int i=0; i<this.dataDimension; i++){
			for(int j=0; j<this.sampleNum; j++){
				A.set(i, j, A.get(i, j) - mean.get(i, 0));
			}
		}
		//create svd 
		SingularValueDecomposition<DenseMatrix64F> svd = 
				DecompositionFactory.svd(this.dataDimension, this.sampleNum, true, false, true);
		if(! svd.decompose(A))
			throw new RuntimeException("SVD failed");
		
		svd.getU(this.prinComps, false);
		svd.getW(this.singValue);
		
		SingularOps.descendingOrder(this.prinComps, false, this.singValue, null, false);
	
		
		//use cross product to force the right-hand rule on plane basis
		//L_R_O, wrapper of prinComps
		SimpleMatrix u = L_R_O.extractVector(false, 0);//1st column
		SimpleMatrix v = L_R_O.extractVector(false, 1);//2nd column
		SimpleMatrix n = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, 1));
		L_R_O.setColumn(2, 0, 
				u.get(1)*v.get(2) - u.get(2)*v.get(1),
				u.get(2)*v.get(0) - u.get(0)*v.get(2),
				u.get(0)*v.get(1) - u.get(1)*v.get(0));//cross product of u and v for 3rd column
				
		//reverse the y,z vectors if angle between plane_z_axis and lidar_z_axis is larger than 90 degree
		if(this.prinComps.get(2,2) < 0){
			L_R_O.setColumn(1, 0, 
			-v.get(0), -v.get(1), -v.get(2));
			L_R_O.setColumn(2, 0, 
			-L_R_O.get(0, 2), -L_R_O.get(1, 2), -L_R_O.get(2, 2));
		}
		
		//form proj_R_O, vector O_proj is the projection of O_L
		//so the columns of proj_R_O can be derived from L_R_O by simple manipulation
		DenseMatrix64F O_X_L = L_R_O.extractVector(true, 0).getMatrix();
		NormOps.normalizeF(O_X_L);
		DenseMatrix64F O_Y_L = L_R_O.extractVector(true, 1).getMatrix();
		NormOps.normalizeF(O_Y_L);
		//L_R_O -> proj_R_O, proj is the new coordinate frame from Lidar frame, parallel to O
		proj_R_O.setRow(0, 0, O_X_L.get(0), O_X_L.get(1), 0);
		proj_R_O.setRow(1, 0, O_Y_L.get(0), O_Y_L.get(1), 0);
		proj_R_O.setRow(2, 0, 0, 0, 1);
		
		//form proj_R_L
		proj_R_L = proj_R_O.mult(L_R_O.transpose());
		
		//O_proj = O_R_L * (-L_O) | z=0
		O_proj = L_R_O.transpose().mult(o.negative()); O_proj.set(2, 0); //z=0
		//L_proj = L_O + L_R_O * O_proj
		L_proj = o.plus(L_R_O.mult(O_proj));
	}
	
	
	//transform point from lidar to ground plane frame
	public Point3D lidar2Plane(Point3D p){
		SimpleMatrix L_p = SimpleMatrix.wrap(new DenseMatrix64F(this.dataDimension, 1));
		L_p.setColumn(0, 0, p.x, p.y, p.z);
		
		//proj_p = proj_R_L * (L_p - L_proj)
		SimpleMatrix proj_p = proj_R_L.mult(L_p.minus(L_proj));
		
		return new Point3D(proj_p.get(0), proj_p.get(1), proj_p.get(2));
	}
	
	//transform point from ground plane frame to lidar frame
	public Point3D plane2Lidar(Point3D p){
		SimpleMatrix proj_p = SimpleMatrix.wrap(new DenseMatrix64F(this.dataDimension, 1));
		proj_p.setColumn(0, 0, p.x, p.y, p.z);
		
		//L_p = L_R_proj * proj_P + L_proj
		SimpleMatrix L_p = proj_R_L.transpose().mult(proj_p).plus(L_proj);
		return new Point3D(L_p.get(0), L_p.get(1), L_p.get(2));
	}
	
	public void setGridRange(double xMin, double xMax, int xNum, double yMin, double yMax, int yNum){
		double[] xgrid = new double[xNum*2+yNum*2];
		double[] ygrid = new double[xNum*2+yNum*2];
		double[] zgrid = new double[xNum*2+yNum*2];
		//fill the grids by the order
		//(x0,yMin),(x0,yMax),(x1,yMin),(x1,yMax)...
		//(xMin,y0),(xMax,y0),(xMin,y1),(xMax,y1)...
		double xRes=(xMax-xMin)/(xNum-1);
		double yRes=(yMax-yMin)/(yNum-1);
		int i=0;
		for(double x=xMin; x<=xMax; x+=xRes){
			xgrid[i]=x;
			ygrid[i]=yMin;
			i++;
			xgrid[i]=x;
			ygrid[i]=yMax;
			i++;
		}
		for(double y=yMin; y<=yMax; y+=yRes){
			xgrid[i]=xMin;
			ygrid[i]=y;
			i++;
			xgrid[i]=xMax;
			ygrid[i]=y;
			i++;
		}
		
		grid = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, xgrid.length));
		for(i=0; i<xgrid.length; i++){
			grid.setColumn(i, 0, xgrid[i], ygrid[i], zgrid[i]);
		}
		
		L_xgrid = SimpleMatrix.wrap(new DenseMatrix64F(1, xgrid.length));
		L_ygrid = SimpleMatrix.wrap(new DenseMatrix64F(1, xgrid.length));
		L_zgrid = SimpleMatrix.wrap(new DenseMatrix64F(1, xgrid.length));
	}
	
	//get gird end points to draw grids
	public FloatBuffer getPlaneGrid(){
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.grid.getNumElements());//xgrid+ygird+zgrid
		//transform xgrid, ygrid, zgrid to Lidar frame
		this.transformGrid2Lidar(); 
		//fill fb by (x,y,z - x,y,z) so they can be drawn as a line
		for(int i=0; i<this.grid.numCols(); i++){
			fb.put((float)L_xgrid.get(i));
			fb.put((float)L_ygrid.get(i));
			fb.put((float)L_zgrid.get(i));
		}
		fb.rewind();
		return fb;
	}
	
	public int getGridPointNum(){
		return this.grid.numCols();
	}
	
	public void printPlaneVectors(){
		System.out.printf("sample number %d\n", this.sampleNum);
		System.out.println("plane origin: ");
		o.print();
		System.out.println("O_R_L: ");
		L_R_O.print();
		
		System.out.println("proj_R_L: ");
		proj_R_L.print();
		System.out.println("lidar proj: ");
		L_proj.print();
		System.out.println("proj_R_O: ");
		proj_R_O.print();
	}

	//transform grid x,y,z to lidar frame
	private void transformGrid2Lidar(){
		//L_p = L_R_proj * proj_p - L_proj
		SimpleMatrix L_grid = proj_R_L.transpose().mult(grid);
		CommonOps.add(L_grid.extractVector(true, 0).getMatrix(), L_proj.get(0), L_xgrid.getMatrix());
		CommonOps.add(L_grid.extractVector(true, 1).getMatrix(), L_proj.get(1), L_ygrid.getMatrix());
		CommonOps.add(L_grid.extractVector(true, 2).getMatrix(), L_proj.get(2), L_zgrid.getMatrix());
	}
	
	public double calcVectorAngle(Point3D vector){
		SimpleMatrix vec = SimpleMatrix.wrap(new DenseMatrix64F(dataDimension, 1));
		vec.setColumn(0, 0, vector.x, vector.y, vector.z);
		double dot = proj_R_L.extractVector(true, 2).dot(vec);
		dot/=vec.normF();
		return Math.acos(dot);
	}
	
	public Point3D getSingularValues(){
		return new Point3D(this.singValue.get(0,0), this.singValue.get(1,1), this.singValue.get(2,2));
	}
	
	public Point3D getNormVector(){
		return new Point3D(this.proj_R_L.get(2,0), this.proj_R_L.get(2,1), this.proj_R_L.get(2,2));
	}
	//get ground frame w.r.t lidar frame
	public CoordinateFrame getGroundFrameInLidar(){
		CoordinateFrame gf = new CoordinateFrame();
		gf.copyRotation(proj_R_L);
		return gf;
	}
	//get ground frame w.r.t world frame
	public CoordinateFrame getGroundFrameInWorld(CoordinateFrame bf){
		CoordinateFrame gf = new CoordinateFrame();
		gf.copyRotation(bf.w_R.mult(proj_R_L.transpose()));
		gf.copyTranslation(bf.getTranslation().plus(bf.w_R.mult(L_proj)));
		return gf;
	}
}
