package detection;

import java.util.Properties;

public class TrackConf {
    public int numOfParticles;

    public double s_occl;
    public double s_bound;
    public double s_occupy;
    public double s_free;
   
    public double d_occupy;
    public double d_bound;

    public double angleRateLowSpeed;
    public double angleRateHighSpeed;
    public double acceleratePcrt;
    public double minSpeed;
    public double minAcc;
    
    public int maxHitThres=5;
	public double ratioNumOfHit=0.05;
	public int maxSeqLost=3;

    public TrackConf(Properties conf){
    	numOfParticles = Integer.parseInt(conf.getProperty("numOfParticles"));
    	
    	s_occl = Double.parseDouble(conf.getProperty("s_occl"));
        s_bound = Double.parseDouble(conf.getProperty("s_bound"));
        s_occupy = Double.parseDouble(conf.getProperty("s_occupy"));
        s_free = Double.parseDouble(conf.getProperty("s_free"));
       
        d_occupy = Double.parseDouble(conf.getProperty("d_occupy"));
        d_bound = Double.parseDouble(conf.getProperty("d_bound"));

        angleRateLowSpeed = Double.parseDouble(conf.getProperty("angleRateLowSpeed"));
        angleRateHighSpeed = Double.parseDouble(conf.getProperty("angleRateHighSpeed"));
        acceleratePcrt = Double.parseDouble(conf.getProperty("acceleratePcrt"));
        minSpeed = Double.parseDouble(conf.getProperty("minSpeed"));
        minAcc = Double.parseDouble(conf.getProperty("minAcc"));
        
        maxHitThres = Integer.parseInt(conf.getProperty("maxHitThres"));
        ratioNumOfHit = Double.parseDouble(conf.getProperty("ratioNumOfHit"));
        maxSeqLost = Integer.parseInt(conf.getProperty("maxSeqLost"));
    }
}
