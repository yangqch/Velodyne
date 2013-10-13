package application;

import gridmap_generic.GridmapMatrix;
import gridmap_generic.HeightCell;
import gridmap_generic.OccupyCell;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;


import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;
import VelodyneView.LidarFrameProcessor;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

public class DataLoggerWithOccupyMap {
	
	public static void main(String argv[]){
		String raw_data_filename = "";
		String dst_data_filename = "";
		String occupy_map_dir = "";
		double occupyProb=0.5;
		try{
			raw_data_filename = argv[0];
			dst_data_filename = argv[1];
			occupy_map_dir = argv[2];
			occupyProb = Double.parseDouble(argv[3]);
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar raw_data_filename dst_data_filename occupy_map_filename occupyProb(prob larger than this thres will be considered as occupied)\n" +
					"dump motion points in each frame");
			System.exit(-1);
		}
		
		log_data_using_occupy_map(raw_data_filename, dst_data_filename, occupy_map_dir, occupyProb);
	}
	
	static void log_data_using_occupy_map(String raw_data_filename, String dst_data_filename, String occupy_map_dir, double occupyProb){
		DataLogger vtLogger = new DataLogger();
//		FrameTransformer mTransformer = new FrameTransformer();
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
			GridmapMatrix<OccupyCell> matrix = GridmapMatrix.loadGridmapMatrix(new File(occupy_map_dir), new OccupyCell(), true);
			dos = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(new File(dst_data_filename))));
			
			lfFac=new LidarFrameFactory(inputFile);
			processor = new LidarFrameProcessor(lfFac, null, matrix);//lidar frame factory, height map, occupy map
			processor.getReady(0);//start from time 0
				
			int cnt=0;
			while(true){
				processor.filterWithOccupyGridmap(occupyProb, null);//filter with no mask
				//log data
				//$(1char)_time(1f)_LidarFrame(12f)_numOfPoint(1long)_Points(3*numOfPoint float)
				pointCnt+=vtLogger.logVirtualTable(dos, processor.getCurFrame(), processor.getVirtualTable(), processor.getOccupyMapFilter().getMask());//log with motion mask
				//flush every 100 frames
				cnt++;					
				if(cnt%100==0){
					dos.flush();
					System.out.printf("total point logged %d\n", pointCnt);
					pointCnt=0;
				}
				processor.readNextFrame();
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

}
