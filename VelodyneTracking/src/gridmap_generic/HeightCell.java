package gridmap_generic;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Collections;
import java.util.PriorityQueue;
import java.util.concurrent.ExecutionException;

import org.omg.CORBA.portable.Streamable;

/**
 * Cell for height map, average the lowest k points stored in heap
 * @author qichi
 *
 */
public class HeightCell implements Cell{
	transient PriorityQueue<Float> heap;//a max-heap to store least k cells
	int capacity;//number of least cells in heap
	float height;//final height averaged by all elements in heap
	
	public HeightCell(int maxHeapSize) {
		this.capacity = maxHeapSize < 1 ? 1 : maxHeapSize;
		heap = new PriorityQueue<Float>(this.capacity);
		height = Float.MIN_VALUE;
		heap.offer(height);
	}
	
	@Override
	public double getValue() {
		return height;
	}
	
	/**
	 * add cell to heap if cell smaller than top of heap
	 */
	@Override
	public void update(double value) {
		if(heap.size()>=capacity){
			if(value > heap.peek()){
				heap.poll();
			}else{
				return;
			}
		}
		heap.offer((float)value);
	}

	@Override
	public void reset() {
		heap = new PriorityQueue<Float>(capacity);
	}

	@Override
	public Cell cloneCell() {
		HeightCell c = new HeightCell(this.capacity);
		return c;
	}
	
	@Override
	public String toString() {
		StringBuilder sb=new StringBuilder();
		sb.append("height ");sb.append(height);sb.append('\n');
		sb.append("heap ");sb.append(this.capacity);sb.append('\n');
		while(!heap.isEmpty()){
			sb.append(heap.poll());sb.append(',');
		}
		sb.append('\n');
		return sb.toString();
	}
	
	private synchronized void writeObject(java.io.ObjectOutputStream stream) throws IOException {
		float sum=0; int size =heap.size();
		while(!heap.isEmpty()){
			sum+=heap.poll();
		}
		this.height = sum/size;
		
		stream.defaultWriteObject();
	}
	
	private void readObject(java.io.ObjectInputStream in) throws IOException, ClassNotFoundException{
		in.defaultReadObject();
		heap = new PriorityQueue<Float>(this.capacity);
		heap.add(this.height);
	}
	
	public static void main(String[] argv){
		HeightCell hc1 = new HeightCell(10);
		for(int i=20; i>0; i--){
			hc1.update(i);
		}		
		HeightCell hc2 = new HeightCell(10);
		for(int i=0; i<20; i++){
			hc2.update(i);
		}
		HeightCell hc3 = new HeightCell(0);

		
		try{
			FileOutputStream out = new FileOutputStream(new File("./HeightCell"));
			ObjectOutputStream os = new ObjectOutputStream(out);
			os.writeObject(hc1);os.writeObject(hc2);os.writeObject(hc3);
			out.close(); os.close();
			System.out.println(hc1);System.out.println(hc2);System.out.println(hc3);
			
			FileInputStream in = new FileInputStream(new File("./HeightCell"));
			ObjectInputStream is = new ObjectInputStream(in);
			hc1=(HeightCell)is.readObject();hc2=(HeightCell)is.readObject();hc3=(HeightCell)is.readObject();
			System.out.println(hc1);System.out.println(hc2);System.out.println(hc3);
			in.close(); is.close();
		}catch(Exception e){
			e.printStackTrace();
		}finally{

		}
	}
}