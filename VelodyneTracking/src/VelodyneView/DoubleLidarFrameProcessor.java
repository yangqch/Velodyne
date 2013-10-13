package VelodyneView;

import java.io.EOFException;

import prepocess.ConnCompFilter;
import prepocess.HeightMapFilter;
import prepocess.OccupyMapFilter;
import prepocess.RangeFilter;
import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.VirtualTable;

public class DoubleLidarFrameProcessor {
	private LidarFrameFactory mainFrameFac, motionFrameFac;
	private LidarFrame mainFrame, motionFrame;
	private VirtualTable mainTable, motionTable;
	
	boolean[][] mask1, mask2, mask;
	
	private ConnCompFilter connCompFilter;
	
	public DoubleLidarFrameProcessor(LidarFrameFactory mainFrameFac, LidarFrameFactory motionFrameFac) {
		this.mainFrameFac = mainFrameFac;
		this.motionFrameFac = motionFrameFac;
	}
	
	public void getReady(float startTime) throws Exception{
		try{
			mainFrame = this.mainFrameFac.getLidarFrame();
			motionFrame = this.motionFrameFac.getLidarFrame();
			while(mainFrame.getTime()<startTime){
				mainFrame = this.mainFrameFac.getLidarFrame();
				motionFrame = this.motionFrameFac.getLidarFrame();
			}

			this.mainTable = new VirtualTable();
			VirtualTable.convertLidarFrame(mainFrame, this.mainTable);
			
			
			this.motionTable = new VirtualTable();
			VirtualTable.convertLidarFrame(motionFrame, this.motionTable);
			
			this.connCompFilter = new ConnCompFilter(this.mainTable);//only use table size
			
			
		}catch(EOFException e){
			System.out.printf("no frame later than startTime %.3f\n", startTime);
			throw e;
		}
	}
	
	public void readNextFrame() throws Exception{
		this.mainFrame = this.mainFrameFac.getLidarFrame();
		this.mainTable = new VirtualTable();
		VirtualTable.convertLidarFrame(this.mainFrame, this.mainTable);
			
		this.motionFrame = this.motionFrameFac.getLidarFrame();
		this.motionTable = new VirtualTable();
		VirtualTable.convertLidarFrame(this.motionFrame, this.motionTable);
		
		//System.out.printf("main local: %s, body: %s\n", mainFrame.getLocalWorldFrame(), mainFrame.getBodyFrame());
		//System.out.printf("motion local: %s, body: %s\n", motionFrame.getLocalWorldFrame(), motionFrame.getBodyFrame());
	}
	
	public boolean[][] filterStaticPoints() throws Exception{
		boolean[][] mask = mainTable.makeMask(motionTable.getPoint3D(null, false));//the points in staticTable will be masked in mainTable
		//this.reverseMask(mask);
		return mask;
	}
	
	public boolean[][] filterStaticComp(double conn_thres, int numOfPoints, double static_pcrt) throws Exception{
		this.connCompFilter.findConnComp(mainTable, null, conn_thres, numOfPoints);
		boolean[][] motionMask = mainTable.makeMask(motionTable.getPoint3D(null, false));//the points in motionTable will be masked in mainTable
		this.reverseMask(motionMask);//now static mask
		this.connCompFilter.filterStaticComp(static_pcrt, motionMask);//static portion smaller than static_pcrt will be classified as motion
		
		return this.connCompFilter.getDynamicCompMask();
	}
	
	public boolean[][] findConnComp(double conn_thres, int numOfPoints, boolean[][] mask){
		this.connCompFilter.findConnComp(mainTable, mask, conn_thres, numOfPoints);
		return this.connCompFilter.getCompMask();
	}
	
	private void reverseMask(boolean[][] mask){
		for(boolean[] tmp: mask){
			int i=tmp.length-1;
			for(; i>=0; i--){
				tmp[i]=!tmp[i];
			}
		}
	}
	
	public VirtualTable getMainTable(){
		return this.mainTable;
	}
	
	public VirtualTable getMotionTable(){
		return this.motionTable;
	}
	
	public LidarFrame getMainFrame(){
		return this.mainFrame;
	}
	
	public LidarFrame getCurFrame(){
		return this.mainFrame;
	}
	
	public LidarFrame getMotionFrame(){
		return this.motionFrame;
	}
	
	public boolean[][] getMask1(){
		return this.mask1;
	}

	public boolean[][] getMask2(){
		return this.mask2;
	}
	
	public boolean[][] getMask(){
		return this.mask;
	}
	
	public void stop(){
		try{
			this.mainFrameFac.close();
			this.motionFrameFac.close();
		}catch(Exception e){
			e.printStackTrace();
		}
	}
	
}
