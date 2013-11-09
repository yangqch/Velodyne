'''
Created on Dec 24, 2012

@author: qichi
'''
import struct
import sys
import os
import time
from xml.dom import minidom
from xml.dom import *
from math import pi, cos, sin
from collections import namedtuple
import pylab as pl
import scipy.io 
import pdb

HEADER_SIZE=16#char(1->4)+int(4)+double(8)=16
DEG2RAD=pi/180
RAD2DEG=180/pi
CM2METER=0.01
#start and end time stamp
LIDAR_START=20
LIDAR_END=1000


class LidarShot(object):
    info_s=struct.Struct('HB')
    dataLen=3
    def __init__(self, shotBuf):
        self.dist, self.intensity = LidarShot.info_s.unpack(shotBuf)
    
    def __repr__(self):
        return "(%d, %d)"%(self.dist, self.intensity)
    
class LidarFire(object):
    info_s=struct.Struct('HH')#info(block and rotation), 4 bytes
    infoLen=4
    dataLen=100#total 100 bytes, 4 bytes info + 96(3*32) bytes shot data
    shotNum=32#number of shots
    def __init__(self, fireBuf):
        block,self.rot=LidarFire.info_s.unpack(fireBuf[:LidarFire.infoLen])
        self.rot=float(self.rot)/100
        if block==61183: 
            self.block='H'
        elif block==56831: 
            self.block='L'
        else:
            self.block=None
        idx=LidarFire.infoLen
        self.shots=[LidarShot(fireBuf[idx+i*LidarShot.dataLen:idx+(i+1)*LidarShot.dataLen]) for i in range(LidarFire.shotNum)]
    
    def __repr__(self):
        return "%s,%s,%s"%(self.block, self.rot,self.shots)


class LidarScan(object):
    '''
    store 64-length self.dist and self.intensity
    and rotation position degree self.rotPosDeg
    and time...FIXME
    '''
    BAD_POINT=(0,0,0)
    
    def __init__(self, highBlockFire, lowBlockFire, timestamp):
        assert(highBlockFire.rot==lowBlockFire.rot)
        self.timestamp=timestamp
        self.rotPosDeg=highBlockFire.rot
        self.dist=[]
        self.dist.extend([shot.dist for shot in highBlockFire.shots])
        self.dist.extend([shot.dist for shot in lowBlockFire.shots])
        self.intensity=[]
        self.intensity.extend([shot.intensity for shot in highBlockFire.shots])
        self.intensity.extend([shot.intensity for shot in lowBlockFire.shots])
        
        self.dataArray=None
        self.intensityArray=None
        
    def hasValidData(self):
        return self.dataArray!=None and self.intensityArray!=None and len(self.dataArray)>0 and len(self.intensityArray)>0
    
    def setCorrPoints(self, points):
        '''
        points should be a n*4-dim tuple list of 3d coordinates
        '''
        allPoints = pl.asarray([p for p in points if p[:3]!=LidarScan.BAD_POINT])
        
        if len(allPoints)<=0:
#            print points
            print self.dist
#            raise Exception('no valid points')
            return
        
        if len(allPoints.shape)<2:
            print self.points
            raise Exception('LidarScan points has 1-dim shape')
        
        if allPoints.shape[1]!=4: 
            raise Exception('LidarScan should receive corrected points with 4 dimension')
        
        self.dataArray = allPoints[:,:3]
        self.intensityArray = allPoints[:,3:]
        
        
    def __repr__(self):
        return "rotation %.2f degree, contains %d shots"%(self.rotPosDeg, len(self.dist))

class LidarFrame(object):
    '''
    store 360 degree scans and transform the data point w.r.t medium vehicle state
    stateArray---numpy.array, n*6, x,y,z,rx,ry,rz, n is the same as scanList length
    scanList---LidarScan, each has dist list, intensity list and points list
    '''
    headStruct = struct.Struct('>cif6f')
    dataStruct = struct.Struct('>4f')
    
    b_R_L=pl.matrix([[0,1,0],[1,0,0],[0,0,-1]])
    b_T_L=pl.matrix([0,0,0.15]).T
    
    def __init__(self, scanList, stateArray):
        if len(scanList)!=stateArray.shape[0]:
            raise Exception('number of scan and state should be the same')
        times = [scan.timestamp for scan in scanList]
        self.avgTime = times[int(pl.floor(len(times)/2))]
        #self.avgTime = pl.mean([scan.timestamp for scan in scanList])
        
        #transform the 3d coordinates of each scan
        #and numpy.vstack all the output m*3 array together
        
        #find average Lidar frame
        avgBodyState = stateArray[int(pl.floor(len(stateArray)/2))]
        #avgBodyState=pl.mean(stateArray, 0)
        
        
        w_R_avg_L, w_T_avg_L = self._bodyState2LidarState(avgBodyState)
        self.avgLidarState = self._matrix2State(w_R_avg_L, w_T_avg_L)
        
        transform = self._transformPointsFromBodyToAvgLidar                
        #from data points with transformation to avgState
        self.dataPoints = pl.vstack([transform(scan.dataArray, state, w_R_avg_L, w_T_avg_L)  for scan, state in zip(scanList, stateArray) if scan.hasValidData()])
        self.intensity = pl.vstack([scan.intensityArray for scan in scanList if scan.hasValidData()]).flatten()
        
        if self.dataPoints.shape[0]!=self.intensity.shape[0]:
            raise Exception('dist and intensity have different size')
        #from data points without any transformation
        #self.dataPoints=pl.vstack([scan.points for scan in scanList if scan.hasValidData()])
    
    def _transformPointsFromBodyToAvgLidar(self, points, bodyState, w_R_avg_L, w_T_avg_L):
        w_R_L, w_T_L = self._bodyState2LidarState(bodyState)
        return self._transformPoints(points, w_R_L, w_T_L, w_R_avg_L, w_T_avg_L)

    def _matrix2State(self, R, T):
        '''
        calculate euler angle from rotation matrix
        and form the vector state from angle and w_T_L
        '''
        if R[2,0]!=1 or R[2,0]!=-1:
            pitch=-pl.arcsin(R[2,0])#another solution is pi-pitch
            roll=pl.arctan2(R[2,1]/cos(pitch), R[2,2]/cos(pitch))
            yaw=pl.arctan2(R[1,0]/cos(pitch), R[0,0]/cos(pitch))
        else:
            yaw=0
            if R[2,0]==-1:
                pitch=pi/2
                roll=yaw+pl.arctan2(R[0,1],R[0,2])
            else:
                pitch=-pi/2
                roll=-yaw+pl.arctan2(-R[0,1],-R[0,2])
        
        state = pl.array([T[0],T[1],T[2],roll*RAD2DEG,pitch*RAD2DEG,yaw*RAD2DEG])
                
        return state
    
    def _bodyState2LidarState(self, bodyState):
        '''
        transform body 6-dim vector state to lidar matrix state
        return w_R_L(3*3 matrix), w_T_L(3*1 matrix)
        '''
        bodyState[3:]*=DEG2RAD;
        roll=bodyState[3];pitch=bodyState[4];yaw=bodyState[5];
        w_R_b=pl.matrix([[ cos(pitch)*cos(yaw),-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw)],
                         [ cos(pitch)*sin(yaw), cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw),-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw)],
                         [-sin(pitch),           sin(roll)*cos(pitch),                                  cos(roll)*cos(pitch)]])
        w_R_L = w_R_b * LidarFrame.b_R_L 
        w_T_b = pl.matrix(bodyState[:3]).T
        w_T_L = w_T_b + w_R_b * LidarFrame.b_T_L
        return w_R_L, w_T_L
    
    def _transformPoints(self, points, w_R_old, w_T_old, w_R_new, w_T_new):
        '''
        transform lidar points from old state to new state
        Input:
        new/old R/L are both Lidar state
        
        points are n*3-dim array
        return n*3-dim array
        '''
        
        #verify rotation matrix
        #print pl.norm(w_R_old[0],2), pl.norm(w_R_old[1],2), pl.norm(w_R_old[2],2)
        #print pl.norm(w_R_new[0],2), pl.norm(w_R_new[1],2), pl.norm(w_R_new[2],2)
        
        new_R_old = w_R_new.T * w_R_old
        new_T_old = w_R_new.T * (w_T_old - w_T_new)
        
        #pdb.set_trace()
        
        #old points first transform to body frame
        old_P = pl.matrix(points).T
        new_P = new_R_old * old_P + new_T_old
        
        #print new_P.T
        #pdb.set_trace()
        
        return pl.array(new_P.T)
    
    def plotXYZ(self):
        x=self.dataPoints[:,0]
        y=self.dataPoints[:,1]
        z=self.dataPoints[:,2]
        pl.subplot(3,1,0)
        pl.plot(x,'.-')
        pl.subplot(3,1,1)
        pl.plot(y,'.-')
        pl.subplot(3,1,2)
        pl.plot(z,'.-')
        pl.savefig('./frame_xyz.png')
    
    def toLogStr(self):
        length=len(self.dataPoints)
        header = LidarFrame.headStruct.pack('$',length,self.avgTime,*self.avgLidarState)
        data=''.join([LidarFrame.dataStruct.pack(point[0], point[1], point[2], intensity) for point, intensity in zip(self.dataPoints, self.intensity)])
        return ''.join([header, data])
        
        
class LidarPacket(object):
    dataLen=1206#total packet size, 1200 fire data + 6 info
    fireNum=12#12 fires, each with 100 bytes
    info_s=struct.Struct('IBB')#info follows the fire data, from 1200 to 1206
    infoLen=6

    def __init__(self, packetBuf):
        idx=0
        #read fires first, then gps and status
        self.fires=[LidarFire(packetBuf[idx+i*LidarFire.dataLen:idx+(i+1)*LidarFire.dataLen]) for i in range(LidarPacket.fireNum)]
        self.gps, self.status_type, self.status_value = LidarPacket.info_s.unpack(packetBuf[idx:idx+LidarPacket.infoLen])
    
    def getScans(self, last_tm, tm):
        #combine high block and low block
        return map(LidarScan, self.fires[::2], self.fires[1::2], pl.linspace(last_tm,tm, len(self.fires)/2)) 
        
    def __repr__(self):
        return '\n'.join(map('{0}'.format, self.fires))

CaliParam = namedtuple('cali_param',('id','rotCorr','vertCorr','distCorr','distCorrX','distCorrY','vertOffsetCorr','horizOffsetCorr','focalDist','focalSlope'))
#rotCorr   degree
#vertCorr  degree
#distCorr  cm
#distCorrX cm
#distCorrY cm
#vertOffsetCorr cm
#horizOffsetCorr cm
#focalDist 
#focalSlope
CaliTempParam = namedtuple('cali_temp_param',('cosVertAngle', 'sinVertAngle', 'cosRotCorr', 'sinRotCorr', 'vOffsetCorr', 'hOffsetCorr'))

class Lidar_Calib(object):
    def __init__(self, cali_fn):
        self.laserNum=64
        self.minIntensity=0
        self.maxIntensity=255
        self.distLSB=0.2
        self.caliParams=self._readCali(cali_fn)
        if len(self.caliParams)!=self.laserNum: raise Exception('bad laser number(%d) in calibration, it should be %d!'%(len(self.caliParams), self.laserNum))
        self.caliTempParam=self._prepareTempParam()

    def _readCali(self, cali_fn):
        #read attribute from calibration file
        #form the namedtuple and put 64 results into a list
        xmldom=minidom.parse(cali_fn)
        nodes = xmldom.getElementsByTagName('px')
        return [CaliParam._make(map(float, [elem.childNodes[0].data for elem in node.childNodes if elem.nodeType==1])) for node in nodes]

    def _prepareTempParam(self):
        p=[]
        for param in self.caliParams:
            cosVertAngle=cos(param.vertCorr * DEG2RAD)
            sinVertAngle=sin(param.vertCorr * DEG2RAD)
            cosRotCorr=cos(param.rotCorr * DEG2RAD)
            sinRotCorr=sin(param.rotCorr * DEG2RAD)
            vOffsetCorr=param.vertOffsetCorr * CM2METER
            hOffsetCorr=param.horizOffsetCorr * CM2METER
            p.append(CaliTempParam(cosVertAngle, sinVertAngle, cosRotCorr, sinRotCorr, vOffsetCorr, hOffsetCorr))
        return p

    def __repr__(self):
        return '\n'.join(map(str, self.caliParams))


class Lidar_Reader(object):
    LASER_NUM=64
    def __init__(self, log_file, cali_file):
        self.log=open(log_file)
        self.heads=struct.Struct('cid')
        self.headLen=16
        self.calib=Lidar_Calib(cali_file)
        
    def getPacket(self):
        header=self.log.read(self.headLen)
        while len(header)>0:
            hdr,length,tm = self.heads.unpack(header)
            data=self.log.read(length)
            try:
                p=LidarPacket(data)
                yield p, tm
                header=self.log.read(self.headLen)
            except Exception as e:
                print 'packet error, ', e
                self.log.close()
                break
        self.log.close()

    def corrMeasurement(self, scan):
        rotPosDeg=scan.rotPosDeg
        scan.setCorrPoints([self._corrPoint(laserNum, dist, rotPosDeg, intensity) for laserNum, (dist,intensity) in enumerate(zip(scan.dist, scan.intensity))])
        #pdb.set_trace()

    def _corrPoint(self, laserNum, dist, rotPosDeg, intensity):
        #correct each point by self.calib
        #return a tuple of (x,y,z)
        
        #rotPosDeg/=100.0 #I did this step when constructing LidarFire
        
        if dist==0:return 0,0,0,-1
        #get measured distance in cm = dist(mm)*calib.distLSB
        dist1 = dist * self.calib.distLSB
        dist = dist1 + self.calib.caliParams[laserNum].distCorr
        #prepare temp variables 
        cosVertAngle=cos(self.calib.caliParams[laserNum].vertCorr * DEG2RAD)
        sinVertAngle=sin(self.calib.caliParams[laserNum].vertCorr * DEG2RAD)
        cosRotCorr=cos(self.calib.caliParams[laserNum].rotCorr * DEG2RAD)
        sinRotCorr=sin(self.calib.caliParams[laserNum].rotCorr * DEG2RAD)
        cosRotAngle=cos(rotPosDeg * DEG2RAD) * cosRotCorr + sin(rotPosDeg * DEG2RAD) * sinRotCorr
        sinRotAngle=sin(rotPosDeg * DEG2RAD) * cosRotCorr - cos(rotPosDeg * DEG2RAD) * sinRotCorr
        vOffsetCorr=self.calib.caliParams[laserNum].vertOffsetCorr * CM2METER
        hOffsetCorr=self.calib.caliParams[laserNum].horizOffsetCorr * CM2METER
        
        #project dist to xy plane
        xyDist=dist * cosVertAngle
        
        xx = abs( xyDist * sinRotAngle - hOffsetCorr * cosRotAngle )
        yy = abs( xyDist * cosRotAngle + hOffsetCorr * sinRotAngle )
        
        #get 2-points calibration values with linear interpolation
        distCorrX = (self.calib.caliParams[laserNum].distCorr - self.calib.caliParams[laserNum].distCorrX) * (xx-240) / (2504-240) + self.calib.caliParams[laserNum].distCorrX 
        distCorrY = (self.calib.caliParams[laserNum].distCorr - self.calib.caliParams[laserNum].distCorrY) * (yy-193) / (2504-193) + self.calib.caliParams[laserNum].distCorrY
        
        dist1*=CM2METER
        distCorrX*=CM2METER
        distCorrY*=CM2METER
        
        #add dist correction in X
        dist = dist1 + distCorrX
        xyDist = dist * cosVertAngle
        
        #X cooredinate
        x = xyDist * sinRotAngle - hOffsetCorr*cosRotAngle
        #add dist correction in Y
        dist = dist1 + distCorrY
        xyDist = dist * cosVertAngle
        #Y cooredinate
        y = xyDist * cosRotAngle + hOffsetCorr*sinRotAngle
        #Z coordinates
        z = dist * sinVertAngle + vOffsetCorr
        
        return (x,y,z, self._corrIntensity(laserNum, intensity, dist))
    
    def _corrIntensity(self, laserNum, intensity, dist):
        #iScale = self.calib.maxIntensity - self.calib.minIntensity
        focalOffset = 256 * (1 - self.calib.caliParams[laserNum].focalDist/13100)**2
        corrIntensity = intensity + self.calib.caliParams[laserNum].focalSlope * pl.absolute(focalOffset - 256*(1-dist/65535)**2)
        
        if corrIntensity<self.calib.minIntensity: corrIntensity = self.calib.minIntensity
        if corrIntensity>self.calib.maxIntensity: corrIntensity = self.calib.maxIntensity
        
        corrIntensity = (corrIntensity-self.calib.minIntensity)/self.calib.maxIntensity
        
        return corrIntensity
        

class StateReader(object):
    def __init__(self, stateFn):
        mat = scipy.io.loadmat(stateFn)['state_out']
        self.timestamp = mat[0,:]
        self.stateMat = mat[1:,:]
        self.cur_idx=0
        self.max_idx = self.timestamp.shape[0]
    
    def getStartTime(self):
        return self.timestamp[0]

    def findState(self, scan):
        while self.isInInterval(scan.timestamp)>0:
            self.cur_idx+=1
            if self.cur_idx+1>=self.max_idx:
                raise Exception('reach the end of state at scan time %f'%scan.timestamp)
            print 'skip to the next state, ', self.getCurInterval()
        if self.isInInterval(scan.timestamp)<0:
            print self.getCurInterval(), ' scan at ', scan.timestamp
            raise Exception('scan pass over the state, error!')
        if self.cur_idx+1>=self.max_idx:
            raise Exception('reach the end of state at scan time %f'%scan.timestamp)
        return self._interpolate(self.cur_idx, self.cur_idx+1, scan.timestamp)
    
    def isInInterval(self, timestamp):
        if timestamp<self.timestamp[self.cur_idx]:
            return -1
        elif timestamp>=self.timestamp[self.cur_idx+1]:
            return 1
        else:
            return 0
    
    def getCurInterval(self):
        return self.timestamp[self.cur_idx],self.timestamp[self.cur_idx+1]
    
    def getNextInterval(self):
        self.cur_idx+=1
        return self.getCurInterval()
    
    def _interpolate(self, start_idx, end_idx, timestamp):
        start_state=self.stateMat[:, start_idx]
        end_state=self.stateMat[:, end_idx]
        start_time=self.timestamp[start_idx]
        end_time=self.timestamp[end_idx]
        return start_state + (end_state - start_state)*(timestamp - start_time)/(end_time - start_time)

class VirtualScanTable2D(object):
    def __init__(self, rotDegRes, vertDegRes, rotDegMin=-180, rotDegMax=180, vertDegMin=-15, vertDegMax=15):
        self.rotDegRes=rotDegRes;self.rotDegMin=rotDegMin;self.rotDegMax=rotDegMax
        self.vertDegRes=vertDegRes;self.vertDegMin=vertDegMin;self.vertDegMax=vertDegMax
        cols=(rotDegMax - rotDegMin)/rotDegRes
        rows=(vertDegMax - vertDegMin)/vertDegRes
        self.valSlots=pl.zeros([rows, cols])
        self.cntSlots=pl.zeros([rows, cols])
        print 'table: ', rows, cols
    
    def fillTable(self, points):
        '''
        point, 3*n array
        '''
        map(self._fillOneSlot, points)

        self.table = self.valSlots[self.cntSlots!=0]/self.cntSlots[self.cntSlots!=0]
    
    def _fillOneSlot(self, point):
        '''
        point, 3-dim array
        '''
        #pdb.set_trace()
        rotDeg = pl.arctan(point[1]/point[0]) * RAD2DEG
        if point[0]<0 and point[1]>0:
            rotDeg+=180
        elif point[0]<0 and point[1]<0:
            rotDeg-=180
        col=pl.floor((rotDeg - self.rotDegMin)/self.rotDegRes)
        
        xyDist=pl.norm(point[:2])
        row=pl.floor((pl.arctan(point[2]/xyDist) * RAD2DEG - self.vertDegMin)/self.vertDegRes)
        
        try:
            self.valSlots[row, col]+=pl.norm(point)
            self.cntSlots[row, col]+=1
        except IndexError:
            exit(0)
    
        return row, col

def plotScanXYZ(scans):
    xyz=[],[],[]
    for scan in scans:
        x,y,z = zip(*scan.points)
        xyz[0].append((min(x),max(x)))
        xyz[1].append((min(y),max(y)))
        xyz[2].append((min(z),max(z)))
    pl.subplot(3,1,0)
    x1,x2 = zip(*xyz[0])
    pl.plot(x1,'-r')
    pl.plot(x2,'-b')
    pl.subplot(3,1,1)
    x1,x2 = zip(*xyz[1])
    pl.plot(x1,'-r')
    pl.plot(x2,'-b')
    pl.subplot(3,1,2)
    x1,x2 = zip(*xyz[2])
    pl.plot(x1,'-r')
    pl.plot(x2,'-b')
    pl.savefig('a.png')
    pl.close()
    
    
def test(lidar):
    #10 seconds data takes 22 seconds to read
    t=time.time()
    frames=[]
    for p, tm in lidar.getPacket():
        for i, scan in enumerate(p.getScans()):
            lidar.corrMeasurement(scan)
            frames.append(scan)
        if tm>10: break
    print 'process %d frames'%(len(frames))
    print 'costs %.2f seconds'%(time.time()-t)
    print 'FPS: %.1f'%(len(frames)/(time.time()-t))
            

#PKM_START=0
#PKM_END=100
#ON_ROAD_START=30
#ON_ROAD_END=780
#
#LIDAR_START=ON_ROAD_START
#LIDAR_END=ON_ROAD_END


#def debug():
#    lidar=Lidar_Reader(data_file, cali_file)
#    cnt=0
#    for p, tm in lidar.getPacket():
#        cnt+=1
#        if cnt>100:
#            break
    
def main():
    #set up file names
    DIR=sys.argv[1]
    velo_file=None
    for fn in os.listdir(DIR):
        if fn.startswith("VELODYNE_log"):
            velo_file=fn; break
    data_file=os.path.join(DIR, velo_file)
    #cali_file="./calibration.xml"
    cali_file=sys.argv[2]
    state_file=os.path.join(DIR, "highrate_state.mat")
    agg_raw_file=os.path.join(DIR,"VELODYNE_agg_raw_road_use_midstate.dat")
    
    for fn in (data_file, cali_file, state_file):
        if not os.path.exists(fn):
            print fn, " doesn't exist"
            sys.exit(-1)
######################################################################################
    lidar=Lidar_Reader(data_file, cali_file)
    
    state_reader = StateReader(state_file)
    fw=open(agg_raw_file, 'w')
    
    #find start point, state usually starts later than lidar
    startTime = state_reader.getStartTime()
    for p, tm in lidar.getPacket():
        if tm>startTime: break
    last_tm=tm
    print 'lidar starts from ', tm, 'state starts from ', startTime
    
##############################################################################################
    t=time.time()
    scanInFrame=[]
    cnt=0#count number of frame
    lastRotDeg=-1
    for p, tm in lidar.getPacket():
        if tm<LIDAR_START: continue
        scans=p.getScans(last_tm, tm)
        
        last_tm=tm
        for scan in scans:
            #rotate 360 degree, aggregate to one frame and dump in file
            if lastRotDeg>=scan.rotPosDeg:
                cnt+=1
                print 'frame %d, contains %d scans from %.2f to %.2f'%(cnt, len(scanInFrame), scanInFrame[0].timestamp, scanInFrame[-1].timestamp)
                print 'angular resolution is %.2f'%(360.0/len(scanInFrame))
                print lastRotDeg, scan.rotPosDeg
                #plotScanXYZ(scanInFrame)
                
                #make a lidar_frame
                frame = LidarFrame(scanInFrame, pl.array([state_reader.findState(scanElem) for scanElem in scanInFrame]))
                #frame.plotXYZ();

                fw.write(frame.toLogStr())
                fw.flush()
                #virtualTable.fillTable(frame.dataPoints)
                
                scanInFrame=[]
                
            lastRotDeg=scan.rotPosDeg
            #calibrate scan and put into frame
            lidar.corrMeasurement(scan)
            scanInFrame.append(scan)
            
        if tm>LIDAR_END: break
    
    diff=time.time()-t
    print 'lidar frame rate'%(cnt/(tm-startTime))
    print 'process %d frames'%(cnt)
    print 'costs %.2f seconds'%(diff)
    print 'FPS: %.1f'%(cnt/diff)
    
    fw.close()            
    
if __name__ == '__main__':
    main()
    #debug()    
