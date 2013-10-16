package VelodyneDataIO;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;

public class LidarFrameFactory {
	DataInputStream reader;
	boolean isRawData;//use getLidarFrame or getLidarFrame2
	FrameTransformer trans;
	
	public LidarFrameFactory(File data_file) throws IOException{
		this.reader = new DataInputStream(new BufferedInputStream(new FileInputStream(data_file)));
		String[] s=data_file.getParent().split("/");
		System.out.println(s[s.length-1]);
		isRawData = (s[s.length-1].equals("data"));
		trans=new FrameTransformer();
	}
	
	public void close(){
		try{
			this.reader.close();
		}catch (Exception e){
			e.printStackTrace();
		}
	}
	
	public LidarFrame getLidarFrame() throws IOException{
		LidarFrame lf = null;
		byte startSymbol = this.reader.readByte();
		if(startSymbol==36){//"$"
			lf = this.getLidarFrame1();
			lf.transfromToLocalWorldFrame(trans);//transform to local world frame, comment this line out if you want local lidar view
		}else if(startSymbol==35){//"#"
			lf = this.getLidarFrame2();
		}else{
			throw new IOException(String.format("%c is not valid start symbol", startSymbol));
		}
		lf.makePoints();
		return lf;
	}
	/**
	 * read raw data point from log file
	 * @param attitudeOrMatrix 6 or 12 elements to represent bodyframe
	 * @return
	 */
	public LidarFrame getLidarFrame1() throws IOException{
		int dataNum = this.reader.readInt();
		float time = this.reader.readFloat();

		BodyFrame bf = BodyFrame.readFromStream(this.reader, true);

		LidarFrame lf = new LidarFrame(time, bf, null, dataNum);
		for(int i=0; i<dataNum; i++){
			lf.putData(i, this.reader.readFloat(),this.reader.readFloat(),this.reader.readFloat(), this.reader.readFloat());
		}
		System.out.printf("system time %.5f\n", time);
		return lf;
	}
	/**
	 * read from transformed lidar frame, which contains bodyFrame and localWorldFrame
	 * @return
	 * @throws IOException
	 */
	public LidarFrame getLidarFrame2() throws IOException{
		float time = this.reader.readFloat();
		
		BodyFrame bodyFrame = BodyFrame.readFromStream(this.reader, false);
		CoordinateFrame localWorldFrame = CoordinateFrame.readFromStream(this.reader);
		
		int numOfPoint = this.reader.readInt();
		LidarFrame lf = new LidarFrame(time, bodyFrame, localWorldFrame, numOfPoint);
		for(int i=0;i<numOfPoint;i++){
			lf.putData(i, this.reader.readFloat(),this.reader.readFloat(),this.reader.readFloat(), VirtualTable.defaultNullIntensity);
		}
		System.out.printf("system time %.5f\n", time);
		return lf;
	}

//	/**
//	 * get lidar frame and calibrated by groundFrame
//	 * @param attitudeOrMatrix
//	 * @param groundFrame
//	 * @param trans
//	 * @return
//	 * @throws IOException
//	 */
//	public LidarFrame getLidarFrame(boolean attitudeOrMatrix, CoordinateFrame groundFrame, FrameTransformer trans) throws IOException{
//		LidarFrame lf = this.getLidarFrame();
//		//transfrom to ground frame and change bodyframe to ground frame
//		lf.transfromBodyFrame(trans, groundFrame);
//		return lf;
//	}

}
