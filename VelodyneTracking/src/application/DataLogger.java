package application;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;


import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import VelodyneView.LidarFrameProcessor;
import calibration.FrameTransformer;

public class DataLogger {
	
	public static void main(String argv[]){
//		findBoundary("/home/qichi/Qichi_Velodyne/processed_data/CA215N/trip1/vt.dat");
//		System.exit(0);
		
		String raw_data_filename = "";
		String dst_data_filename = "";
		boolean frame_or_vt = true;
		try{
			if(argv[0].equals("-f")){
				frame_or_vt = true;
			}else if(argv[0].equals("-v")){
				frame_or_vt = false;
			}else{
				System.out.println(argv[0]);
				throw new Exception("USAGE: java -jar *.jar frame_or_vt(-f or -v) raw_data_filename dst_data_filename");
			}
			raw_data_filename = argv[1];
			dst_data_filename = argv[2];
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar frame_or_vt(-f or -v) raw_data_filename dst_data_filename");
			System.exit(-1);
		}
		
		log_data_as_virtual_table(raw_data_filename, dst_data_filename, frame_or_vt);
	}
	
	static void log_data_as_virtual_table(String raw_data_filename, String dst_data_filename, boolean frame_or_vt){
		DataLogger vtLogger = new DataLogger();
		FrameTransformer mTransformer = new FrameTransformer();
//		CoordinateFrame calibFrame = new CoordinateFrame(new double[] {0,0,0,
//				0.9999800441981136, 0.00037833642524528284, 0.0036987820702358474,
//				0.00037833642524528284, 0.9896323094339619, -0.10125933784433956,
//				-0.003718152332877361, 0.10179006768396229, 0.9947925857075474});

		LidarFrameFactory lfFac = null;
		LidarFrameProcessor processor = null;		
		DataOutputStream dos = null;
		
		int pointCnt=0;
		try{
			File inputFile = new File(raw_data_filename);
			dos = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(new File(dst_data_filename))));

			lfFac=new LidarFrameFactory(inputFile);
			processor = new LidarFrameProcessor(lfFac, null, null);//no occupy map in this application
			processor.getReady(0);//start from time 0
				
			int cnt=0;
			while(true){
				LidarFrame lf = processor.getCurFrame();
				//shouldn't add it here, virtual table has been created in processor.getReady() or processor.readNextFrame()
				//I moved the transformation into LidarFac.getFrame()
				//if it is raw data(start with '$'), the transformation will be taken
				//lf.transfromToLocalWorldFrame(mTransformer);//transform to local world frame, comment this line out in you want local lidar view
				
				//log data
				//$(1char)_time(1f)_bodyFrame(12f)_localWorldFrame(12f)_numOfPoint(1long)_Points(3*numOfPoint float)
				if (frame_or_vt){
					pointCnt+=vtLogger.logRawData(dos, lf);
				}else{
					pointCnt+=vtLogger.logVirtualTable(dos, lf, processor.getVirtualTable(), null);
				}
				
				//flush every 100 frames
				cnt++;					
				if(cnt%100==0){
					dos.flush();
					System.out.printf("total point logged %d\n", pointCnt);
					pointCnt=0;
				}
				processor.readNextFrame();//read next frame
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			BuildHeightMap.checkMemory();
			
			processor.stop();
			try{
				dos.close();
			}catch(Exception e){
				
			}
			System.out.println("finish");
		}
	}
	
	//log raw data in virtual table points
	protected int logVirtualTable(DataOutputStream dos, LidarFrame lf, VirtualTable vt, boolean[][] mask) throws IOException{
		dos.writeByte('#');
		dos.writeFloat(lf.getTime());
		lf.getBodyFrame().printBinaryToFile(dos);//3+9 params
		lf.getLocalWorldFrame().printBinaryToFile(dos);//3+9 params
		
		Point3D[] points = vt.getPoint3D(mask, true);//need max range
		
		dos.writeInt(points.length);
		
		for(int i=0;i<points.length; i++){
			Point3D p = points[i];
			dos.writeFloat((float)p.x);dos.writeFloat((float)p.y);dos.writeFloat((float)p.z);
		}
		return points.length;
	}
	//log raw data in Lidar frame with intensity
	private int logRawData(DataOutputStream dos, LidarFrame lf) throws IOException{
		dos.writeByte('#');
		dos.writeFloat(lf.getTime());
		lf.getBodyFrame().printBinaryToFile(dos);//3+9 params
		lf.getLocalWorldFrame().printBinaryToFile(dos);//3+9 params

		dos.writeInt(lf.getPointNum());
		
		for(int i=0;i<lf.getPointNum(); i++){
			Point3D p = lf.getDataPoint(i);
			dos.writeFloat((float)p.x);dos.writeFloat((float)p.y);dos.writeFloat((float)p.z);
		}
		return lf.getPointNum();
	}
	
	public static void findBoundary(String raw_data_filename){
		LidarFrameFactory lfFac = null;
		LidarFrameProcessor processor = null;		
		
		double min_north=Double.MAX_VALUE;
		double min_east=Double.MAX_VALUE;
		double max_north=-Double.MAX_VALUE;
		double max_east=-Double.MAX_VALUE;
		try{
			File inputFile = new File(raw_data_filename);

			lfFac=new LidarFrameFactory(inputFile);
			processor = new LidarFrameProcessor(lfFac, null, null);//no occupy map in this application
			processor.getReady(0);//start from time 0
			
			while(true){
				LidarFrame lf = processor.getCurFrame();
				double north = lf.getBodyFrame().getPosition().x;
				double east  = lf.getBodyFrame().getPosition().y;
				
				min_north = north<min_north ? north : min_north;
				max_north = north>max_north ? north : max_north;
				min_east  = east<min_east ? east : min_east;
				max_east  = east>max_east ? east : max_east;
				processor.readNextFrame();//read next frame
			}
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			processor.stop();
			System.out.printf("east (%.1f, %.1f), north (%.1f, %.1f)\n", min_east, max_east, min_north, max_north);
			System.out.println("finish");
		}
	}
}

