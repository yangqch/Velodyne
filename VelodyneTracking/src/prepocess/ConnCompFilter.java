package prepocess;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.omg.PortableInterceptor.INACTIVE;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

/**
 * find connected component from virtual table
 * connected components will be labeled on a matrix with same size of virtual table
 * use getMask() to get a boolean mask of all points of connected components
 * @author qichi
 *
 */
public class ConnCompFilter {
	//private VirtualTable vt;
	private int[][] labels;//element-wise same size as virtual table
	private boolean[][] compMask;
	private boolean[][] dynamicCompMask;
	private Map<Integer, ComponentSize> compMap;//valid components and their attributes like size, numOfPoints
	private Set<Integer> dynamicCompSet;//hash set of dynamic component, after static filtering 
	//private Set<Integer> validComp;//store final valid components
	private int rows, cols;
	
//	static double connect_thres=0.5;
	
	public ConnCompFilter(VirtualTable vt) {
		//this.vt=vt;
		rows=vt.getRowNum();
		cols=vt.getColNum();
		labels = new int[rows][cols];
		compMask = new boolean[rows][cols];
		dynamicCompMask = new boolean[rows][cols];
		compMap = new HashMap<Integer, ComponentSize>();
		dynamicCompSet = new HashSet<Integer>();
	}
	/**
	 * find connected components from vt and store results in labels and compMap
	 * @param mask: a mask can be applied, false will be considered as background
	 * 				if mask=null, use all data
	 */
	public void findConnComp(VirtualTable vt, boolean[][] mask, double conn_thres, int numOfPoints){
		ArrayList<SetNode> labelList=new ArrayList<SetNode>();//ith element is ith label, a SetNode, 0th is revered for background
		labelList.add(null);
		//initialize the label matrix, masked is 0, no measurement is -1
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				Point3D p = vt.getPoint3D(i, j);
				double dist = vt.getDistance(i, j);
				if(p==null || dist>=LidarFrame.MAX_RANGE){//null point and max range point will be considered as invalid
					labels[i][j] = -1;//no measurement is not component, mask out as well
				}else{
					labels[i][j]=0;
				}
			}
		}
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				if(mask!=null && !mask[i][j]){
					continue;//mask out
				}
				if(labels[i][j]==-1) continue;
				Point3D p = vt.getPoint3D(i, j);
				//find connected component
				//Set<Integer> neighborLabels = this.findNeighborLabels(vt, i, j, p, conn_thres, labelList);//4 connected neighbor
				Set<Integer> neighborLabels = this.findNeighborLabels(vt, i, j, 3, p, conn_thres, labelList);//n-by-n window
				//Set<Integer> neighborLabels = this.findNeighborLabelsDFS(vt, i, j, p, conn_thres);//DFS
				if(neighborLabels.size()==0){//no neighbors, create new label
					int nextLabel = labelList.size();
					//System.out.printf("new Label %d, (%d,%d)\n", nextLabel, i, j);
					labels[i][j]=nextLabel;
					labelList.add(new SetNode(nextLabel));
				}else{//set min label among neighbors to current one and union min label to all the other neighbor labels
					//System.out.printf("has %d neighbor labels\n", neighborLabels.size());
					int nextLabel = Collections.min(neighborLabels);
					labels[i][j]=nextLabel;
					for(int label : neighborLabels){
						SetNode.Union(labelList.get(label), labelList.get(nextLabel));
					}
				}
			}
		}
		//join labels
		HashMap<Integer, ComponentSize> tmpComps = new HashMap<Integer, ComponentSize>();
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				int label = labels[i][j];
				if (label>0){
					int newLabel = labelList.get(label).find().label;
					labels[i][j]=newLabel;
					if(!tmpComps.containsKey(newLabel)){
						tmpComps.put(newLabel, new ComponentSize(newLabel));
					}
					tmpComps.get(newLabel).addPoint(vt.getPoint3D(i, j));
				}
			}
		}
		System.out.printf("%d lables before comp size filtering\n", tmpComps.size());
		//filter small components with small number of points
		compMap.clear();
		for(Entry<Integer, ComponentSize> entry: tmpComps.entrySet()){
			ComponentSize comp = entry.getValue();
			if (comp.numOfPoint>=numOfPoints){
				compMap.put(entry.getKey(), entry.getValue());
			}
			//if (comp.minZ>0.1 && comp.getXRange()<10 && comp.getYRange()<10) continue;
			//if (comp.getXRange()*comp.getYRange()>100) continue;
		}
		int cnt=0;
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				if (labels[i][j]>0 && compMap.containsKey(labels[i][j])){
					this.compMask[i][j] = true; cnt++;
				}else{
					this.compMask[i][j] = false;
				}
			}
		}
		System.out.printf("%d lables, covered %d points\n", compMap.size(), cnt);
	}
	
	/**
	 * connected component is defined as 8-connected neighborhood, distance<conn_thres
	 * try to change to n*n neighborhood
	 * @param vt
	 * @param row
	 * @param col
	 * @param cur
	 * @param conn_thres
	 * @param labelList
	 * @return
	 */
	private Set<Integer> findNeighborLabels(VirtualTable vt, int row, int col, Point3D cur, double conn_thres, ArrayList<SetNode> labelList){
		//System.out.printf("find neighbor (%d,%d)\n", row, col);
		Set<Integer> L=new HashSet<Integer>();
		//check left, left-up, up, up-right
		if(col>0){//left
			Point3D p=vt.getPoint3D(row, col-1);
			if(labels[row][col-1]>0 && p.calcDist(cur)<conn_thres){
				L.add(this.labels[row][col-1]);
				//System.out.printf("left Label %d\n", this.labels[row][col-1]);
			}
		}
		if(row>0 && col>0){//left-up
			Point3D p=vt.getPoint3D(row-1, col-1);
			if(labels[row-1][col-1]>0 && p.calcDist(cur)<conn_thres){
				L.add(this.labels[row-1][col-1]);
				//System.out.printf("left-up Label %d\n", this.labels[row-1][col-1]);
			}
		}
		if(row>0){//up
			Point3D p=vt.getPoint3D(row-1, col);
			if(labels[row-1][col]>0 && p.calcDist(cur)<conn_thres){
				L.add(this.labels[row-1][col]);
				//System.out.printf("up Label %d\n", this.labels[row-1][col]);
			}
		}//up-right, if reach the last column, check up-0th column(virtual table is a loop on rotation)
		if(row>0 && col<cols){
			int c = col==cols-1 ? c=0 : col+1;
			Point3D p=vt.getPoint3D(row-1, c);
			if(labels[row-1][c]>0 && p.calcDist(cur)<conn_thres){
				L.add(this.labels[row-1][c]);
				//System.out.printf("left Label %d\n", this.labels[row-1][c]);
			}
		}
		return L;
	}
	/**
	 * if there are neighbor(XYDist < conn_thres) within n-by-n matrix, add to labelList
	 * @param vt
	 * @param row
	 * @param col
	 * @param cur
	 * @param conn_thres
	 * @param labelList
	 * @return
	 */
	private Set<Integer> findNeighborLabels(VirtualTable vt, int row, int col, int winSize, Point3D cur, double conn_thres, ArrayList<SetNode> labelList){
		//System.out.printf("find neighbor (%d,%d)\n", row, col);
		Set<Integer> L=new HashSet<Integer>();
		int row_up = row<winSize ? 0 : row-winSize;
		
		for(int i=row_up; i<=row; i++){
			for(int j=col-winSize; j<col+winSize; j++){
				if(i==row && j==col) break;
				if(j<0) continue;
				int jj = j>=cols ? j%cols : j;
				Point3D p=vt.getPoint3D(i ,jj);
				if(labels[i][jj]>0 && p.calcXYDist(cur)<conn_thres){
					L.add(labels[i][jj]);
				}
			}
		}
		return L;
	}
	
	private Set<Integer> findNeighborLabelsDFS(VirtualTable vt, int row, int col, Point3D cur, double conn_thres){
		Set<Integer> L=new HashSet<Integer>();
		boolean[][] visited = new boolean[rows][cols];
		int up = row-1; int down = row+1;
		int left = col==0 ? vt.getColNum()-1 : col-1;
		int right = col==vt.getColNum()-1 ? 0 : col+1;
		visited[row][col]=true;
		DFS(vt, up, col, cur, conn_thres, L, visited, 0);
		DFS(vt, down, col, cur, conn_thres, L, visited, 0);
		DFS(vt, row, left, cur, conn_thres, L, visited, 0);
		DFS(vt, row, right, cur, conn_thres, L, visited, 0);
		return L;
	}
	
	private void DFS(VirtualTable vt, int row, int col, Point3D cur, double conn_thres, Set<Integer> labels, boolean[][] visited, int depth){
		if(row<0 || row>=vt.getRowNum() || visited[row][col] || depth>10){
			return;
		}else{
			visited[row][col]=true;
		}
		//row, col are valid and not visited
		if(this.labels[row][col]==0){//0 means masked
			return;
		}else if(this.labels[row][col]>0){//if has label, calculate distance
			Point3D p = vt.getPoint3D(row, col);
			if(p.calcDist(cur)<conn_thres){
				labels.add(this.labels[row][col]);
			}
			return;
		}else if(this.labels[row][col]==-1){//-1 is no measurement, it means not sure, keep searching
			int up = row-1; int down = row+1;
			int left = col==0 ? vt.getColNum()-1 : col-1;
			int right = col==vt.getColNum()-1 ? 0 : col+1;
			
			DFS(vt, up, col, cur, conn_thres, labels, visited, depth+1);
			DFS(vt, down, col, cur, conn_thres, labels, visited, depth+1);
			DFS(vt, row, left, cur, conn_thres, labels, visited, depth+1);
			DFS(vt, row, right, cur, conn_thres, labels, visited, depth+1);
		}		
	}
	
	/**
	 * filter components with more than pctOfStatic points are masked as static
	 * @param pctOfStatic: dynamic comp contains static points smaller than this thres
	 * @param staticMask: true-static, false-moving
	 */
	public void filterStaticComp(double pctOfStatic, boolean staticMask[][]){
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){//only work for the components in validSet
				if(labels[i][j]>0 && staticMask[i][j] && compMap.containsKey(labels[i][j])){
					compMap.get(labels[i][j]).numOfStaticPoint++;
					//System.out.println("find");
				}
			}
		}
		this.dynamicCompSet.clear();
		for(Entry<Integer, ComponentSize> entry: compMap.entrySet()){
			if(entry.getValue().staticPortion()<pctOfStatic){
				this.dynamicCompSet.add(entry.getKey());
			}
		}
		System.out.printf("dynamic component %d\n", this.dynamicCompSet.size());
		ArrayList<Integer> badCompList = this.cleanupDynamicComp(2, 10);//thickness, minArea
		for(int i: badCompList){
			this.dynamicCompSet.remove(i);
		}
		System.out.printf("after filtering dynamic component %d\n", this.dynamicCompSet.size());
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				this.dynamicCompMask[i][j]= this.dynamicCompSet.contains(labels[i][j]) ? true : false;
			}
		}
	}
	
	private ArrayList<Integer> cleanupDynamicComp(double thickness, double minArea){
		HashMap<Integer, CompStat> compStatMap = new HashMap<Integer, CompStat>();
		for(int label: this.dynamicCompSet){
			compStatMap.put(label, new CompStat(label));
		}
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				if(this.dynamicCompSet.contains(labels[i][j])){
					compStatMap.get(labels[i][j]).addX();
				}
			}
			for(Entry<Integer, CompStat> entry: compStatMap.entrySet()){
				entry.getValue().updateX();
			}
		}
		for(int j=0; j<cols; j++){
			for(int i=0; i<rows; i++){
				if(this.dynamicCompSet.contains(labels[i][j])){
					compStatMap.get(labels[i][j]).addY();
				}
			}
			for(Entry<Integer, CompStat> entry: compStatMap.entrySet()){
				entry.getValue().updateY();
			}
		}
		ArrayList<Integer> badComp = new ArrayList<Integer>();
		for(Entry<Integer, CompStat> entry: compStatMap.entrySet()){
			double thickX = entry.getValue().getAvgXThickness();
			double thickY = entry.getValue().getAvgYThickness();
			//System.out.println(entry.getValue());
			if(thickX<thickness){
				badComp.add(entry.getKey());
			}else if(thickY<thickness){
				badComp.add(entry.getKey());
			}else if(thickX*thickY < minArea){
				badComp.add(entry.getKey());
			}
		}
		return badComp;
	}

	public boolean[][] getCompMask(){
		return this.compMask;
	}
	
	public boolean[][] getDynamicCompMask(){
		return this.dynamicCompMask;
	}
	
	public int[][] getLables(){
		return this.labels;
	}
	
	public boolean isValidComp(int label){
		return this.compMap.containsKey(label);
	}
	
	public static void main(String[] argv){
		CompStat x = new CompStat(1);
		x.addX(); x.addX();
		x.updateX();
		x.addX(); x.addX(); x.addX(); 
		x.updateX();
		System.out.println(x);
	}
}

class CompStat{
	int label;
	private ArrayList<Integer> thicknessX;
	private ArrayList<Integer> thicknessY;
	private int cur_x_cnt;
	private int cur_y_cnt;
	public CompStat(int label) {
		this.label = label;
		thicknessX = new ArrayList<Integer>();
		thicknessY = new ArrayList<Integer>();
		reset();
	}
	
	public void reset(){
		cur_x_cnt=0;
		cur_y_cnt=0;
		thicknessX.clear();
		thicknessY.clear();
	}
	
	public void addX(){
		cur_x_cnt++;
	}
	
	public void addY(){
		cur_y_cnt++;
	}
	
	public void updateX(){
		if(cur_x_cnt>0){
			thicknessX.add(cur_x_cnt);
		}
		cur_x_cnt = 0;
	}
	
	public void updateY(){
		if(cur_y_cnt>0){
			thicknessY.add(cur_y_cnt);
		}
		cur_y_cnt = 0;
	}
	
	public double getAvgXThickness(){
		double sum=0;
		for(int x: thicknessX){
			sum+=x;
		}
		return sum/thicknessX.size();
	}
	
	public double getAvgYThickness(){
		double sum=0;
		for(int y: thicknessY){
			sum+=y;
		}
		return sum/thicknessY.size();
	}
	
	public String toString(){
		return String.format("x %.2f, y %.2f", this.getAvgXThickness(), this.getAvgYThickness());
	}
}

class SetNode{
	int label;
	SetNode parent;
	int rank;
	
	public SetNode(int label) {
		this.label=label;
		parent=this;
		rank=0;
	}
	SetNode find() {
		if(this.parent!=this)
			this.parent=this.parent.find();
		return this.parent;
	}
	static void Union(SetNode x, SetNode y){
		SetNode xRoot=x.find();
		SetNode yRoot=y.find();
		if (xRoot==yRoot) return;
		//x and y are not in same set, merge them.
		if(xRoot.rank < yRoot.rank){
			xRoot.parent=yRoot;
		}else if(xRoot.rank > yRoot.rank){
			yRoot.parent=xRoot;
		}else{
			yRoot.parent=xRoot;
			xRoot.rank++;
		}
	}
}

class ComponentSize{
	int label;
	int numOfPoint;
	double minX, maxX, minY, maxY, minZ, maxZ;
	int numOfStaticPoint;
	public ComponentSize(int label) {
		this.label = label;
		minX=Double.MAX_VALUE;maxX=Double.MIN_VALUE;
		minY=Double.MAX_VALUE;maxY=Double.MIN_VALUE;
		minZ=Double.MAX_VALUE;maxZ=Double.MIN_VALUE;
		numOfStaticPoint = 0; 
		numOfPoint = 0;
	}
	public void addPoint(Point3D p){
		numOfPoint++;
		minX=p.x < minX ? p.x : minX;
		maxX=p.x > maxX ? p.x : maxX;
		minY=p.y < minY ? p.y : minY;
		maxY=p.y > maxY ? p.y : maxY;
		minZ=p.z < minZ ? p.z : minZ;
		maxZ=p.z > maxZ ? p.z : maxZ;
	}
	public double staticPortion(){
		return (double)numOfStaticPoint/(double)numOfPoint;
	}
	public double getXRange(){
		return maxX - minX;
	}
	public double getYRange(){
		return maxY - minY;
	}
	public double getZRange(){
		return maxZ - minZ;
	}
}