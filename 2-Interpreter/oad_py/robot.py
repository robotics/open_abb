'''
Michael Dawson-Haggerty
CMU Robotics Institute
For Dr. David Bourne


robot.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code module SERVER)

NOTES:
For functions which require targets (XYZ positions with quaternion orientation):
Targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

message write loops are funrolled because it's slightly faster and there aren't many elements
'''

import socket, json, os, time
import threading
import collections
    
class Robot:
    def __init__(self, IP='192.168.125.1', PORT=5000, wobj=[[0,0,0],[1,0,0,0]], tool=[[0,0,0], [1,0,0,0]], speed = [100,50,50,50], zone='z5', toolfile=None, zeroJoints = False, verbose=False):
        
        self.BUFLEN = 4096; self.idel = .01
        self.v = verbose

        if verbose: print 'Attempting to connect to ABB robot at', IP
        
        self.robsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robsock.connect((IP, PORT))
        
        if toolfile == None: self.setTool(tool)
        else: setToolFile(toolfile)

        self.setWorkObject(wobj)
        self.setSpeed(speed)
        self.setZone(zone)
        if zeroJoints: 
            self.setJoints()

    def setCartesian(self, pos):
        if len(pos) == 7: pos = [pos[0:3], pos[3:7]]
        if self.checkCoordinates(pos):
            msg = "01 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            if self.v: print 'setCartesian:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else:
            return False

    def setJoints(self, j = [0,0,0,0,90,0]):
        if len(j) == 6:
            msg = "02 " 
            msg = msg + format(j[0], "+08.2f") + " " + format(j[1], "+08.2f") + " " + format(j[2], "+08.2f") + " " 
            msg = msg + format(j[3], "+08.2f") + " " + format(j[4], "+08.2f") + " " + format(j[5], "+08.2f") + " #" 
            if self.v: print 'setJoints:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)  
            return data
        else: return False

    def getCartesian(self):
        msg = "03 #"
        self.robsock.send(msg)
        data = str(self.robsock.recv(self.BUFLEN)).split(' ')
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]

    def getJoints(self):
        msg = "04 #"
        self.robsock.send(msg)
        data = (str(self.robsock.recv(self.BUFLEN)).split(' '))
        r = [float(s) for s in data]
        return r[2:8]

    def getExternalAxis(self):
        msg = "05 #"
        self.robsock.send(msg)
        data = (str(self.robsock.recv(self.BUFLEN)).split(' '))
        print data
        r = [float(s) for s in data]
        return r[2:8]

    def getRobotInfo(self):
        msg = "98 #"
        self.robsock.send(msg)
        data = (str(self.robsock.recv(self.BUFLEN))[5:].split('*'))
        return data

    def setTool(self, tool=[[0,0,0], [1,0,0,0]]):
        #sets the tool object of the robot. 
        #Offsets are from tool0 (tool flange center axis/ flange face intersection)
        if len(tool) == 7: tool = [tool[0:3], tool[3:7]]
        if self.checkCoordinates(tool):
            msg = "06 " 
            msg = msg + format(tool[0][0], "+08.1f") + " " + format(tool[0][1], "+08.1f") + " " + format(tool[0][2], "+08.1f") + " " 
            msg = msg + format(tool[1][0], "+08.5f") + " " + format(tool[1][1], "+08.5f") + " " 
            msg = msg + format(tool[1][2], "+08.5f") + " " + format(tool[1][3], "+08.5f") + " #"    
            if self.v: print 'setTool:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            self.tool = tool
            time.sleep(self.idel)
            return data
        else: return False

    def setToolFile(self, filename):
        if os.path.exists(filename):
            f = open(filename, 'rb');        
            try: tool = json.load(f)
            except: 
                print 'toolfile failed to load!'
                return False
        else: 
            print 'toolfile ', filename, 'doesn\'t exist'
            return False
        self.setTool(tool)
        
    def getTool(self): 
        return self.tool

    def setWorkObject(self, wobj=[[0,0,0],[1,0,0,0]]):
        if len(wobj) == 7: pos = [wobj[0:3], wobj[3:7]]
        if self.checkCoordinates(wobj):
            msg = "07 " 
            msg = msg + format(wobj[0][0], "+08.1f") + " " + format(wobj[0][1], "+08.1f") + " " + format(wobj[0][2], "+08.1f") + " " 
            msg = msg + format(wobj[1][0], "+08.5f") + " " + format(wobj[1][1], "+08.5f") + " " 
            msg = msg + format(wobj[1][2], "+08.5f") + " " + format(wobj[1][3], "+08.5f") + " #"    
            if self.v: print 'setWorkObject:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            time.sleep(self.idel)
            return data
        else: return False
            
    #speed is [linear speed (mm/s), orientation speed (deg/s),
    #          external axis linear, external axis orientation]
    def setSpeed(self, speed=[100,50,50,50]):
        if len(speed) == 4:
            msg = "08 " 
            msg = msg + format(speed[0], "+08.1f") + " " + format(speed[1], "+08.2f") + " "  
            msg = msg + format(speed[2], "+08.1f") + " " + format(speed[3], "+08.2f") + " #"  
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if self.v: print 'setSpeed:', msg 
            time.sleep(self.idel)
            return data
        else: return False
        
    def setZone(self, zoneKey='z1', finep = False, manualZone=[]):
        zoneDict = {'z0': [.3,.3,.03], 'z1': [1,1,.1], 'z5': [5,8,.8], 
                    'z10': [10,15,1.5], 'z15': [15,23,2.3], 'z20': [20,30,3], 
                    'z30': [30,45,4.5], 'z50': [50,75,7.5], 'z100': [100,150,15], 
                    'z200': [200,300,30]}

        #zoneKey: uses values from RAPID handbook (stored here in zoneDict), 'z*' 
        #you should probably use zoneKeys

        #finep: go to point exactly, and stop briefly before moving on

        #manualZone = [pzone_tcp, pzone_ori, zone_ori]
        #pzone_tcp: mm, radius from goal where robot tool center is not rigidly constrained
        #pzone_ori: mm, radius from goal where robot tool orientation is not rigidly constrained
        #zone_ori: degrees, zone size for the tool reorientation

        if finep: zone = [0,0,0]
        else:
            if len(manualZone) == 3: zone = manualZone
            elif zoneKey in zoneDict.keys(): zone = zoneDict[zoneKey]
            else: return False 
        msg = "09 " 
        msg = msg + str(int(finep)) + " "
        msg = msg + format(zone[0], "+08.4f") + " " + format(zone[1], "+08.4f") + " " + format(zone[2], "+08.4f") + " #" 
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        if self.v: print 'setZone:', msg
        time.sleep(self.idel)
        return data

    def addBuffer(self, pos):
        #appends single target to the buffer
        #move will execute at current speed (which you can change between addBuffer calls)
        if len(pos) == 7: pos = [pos[0:3], pos[3:7]]
        if self.checkCoordinates(pos):
            msg = "10 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            if self.v: print 'addBuffer:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            time.sleep(self.idel)
            return data
        else:
            return False
    #adds every position in posList to the buffer
    def setBuffer(self, posList, gotoFirst=False):
       
        self.clearBuffer()
        if self.lenBuffer() <> 0: return False
        for i in posList: self.addBuffer(i)
        if gotoFirst: self.setCartesian(posList[0])
        if self.lenBuffer() == len(posList): return True
        else: 
            self.clearBuffer()
            return False

    def clearBuffer(self):
        msg = "11 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def lenBuffer(self):
        msg = "12 #"
        self.robsock.send(msg)
        data = str(self.robsock.recv(self.BUFLEN)).split(' ')
        return int(float(data[2]))

    #execute every move in buffer as MoveL command (linear move)
    def executeBuffer(self):
        msg = "13 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def setExternalAxis(self, axisValues=[-550,0,0,0,0,0]):
        if len(axisValues) == 6:
            msg = "14 " 
            msg = msg + format(axisValues[0], "+08.2f") + " " + format(axisValues[1], "+08.2f") + " " + format(axisValues[2], "+08.2f") + " " 
            msg = msg + format(axisValues[3], "+08.2f") + " " + format(axisValues[4], "+08.2f") + " " + format(axisValues[5], "+08.2f") + " #" 
            if self.v: print 'setExternalAxis:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)  
            return data
        else: return False

    def setCircular(self, circlePoint, endPoint):
        if len(circlePoint) == 7: circlePoint = [circlePoint[0:3], circlePoint[3:7]]
        if len(endPoint) == 7: endPoint = [endPoint[0:3], endPoint[3:7]]

        if self.checkCoordinates(circlePoint) & self.checkCoordinates(endPoint):
            msg = "15 " 
            msg = msg + format(circlePoint[0][0], "+08.1f") + " " + format(circlePoint[0][1], "+08.1f") + " " + format(circlePoint[0][2], "+08.1f") + " " 
            msg = msg + format(circlePoint[1][0], "+08.5f") + " " + format(circlePoint[1][1], "+08.5f") + " " 
            msg = msg + format(circlePoint[1][2], "+08.5f") + " " + format(circlePoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if data <> '15 1 ': return False

            msg = "16 " 
            msg = msg + format(endPoint[0][0], "+08.1f") + " " + format(endPoint[0][1], "+08.1f") + " " + format(endPoint[0][2], "+08.1f") + " " 
            msg = msg + format(endPoint[1][0], "+08.5f") + " " + format(endPoint[1][1], "+08.5f") + " " 
            msg = msg + format(endPoint[1][2], "+08.5f") + " " + format(endPoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else:
            return False

    def checkCoordinates(self, coords):
        try: 
            if (len(coords) == 2):
                if ((len(coords[0]) == 3) & (len(coords[1]) == 4)): return True
        except: pass
        if self.v: print 'Coordinate check failed on', coords
        return False

    def close(self):
        self.robsock.shutdown(socket.SHUT_RDWR)
        self.robsock.close()

    def __del__(self):
        try: self.close()
        except: pass

class Logger:
    def __init__(self, IP='192.168.125.1', PORT=5001, maxlen=1024, verbose=False):
        self.IP = IP
        self.PORT = PORT
        self.v = verbose

        self.joints = collections.deque(maxlen=maxlen)
        self.cartesian = collections.deque(maxlen=maxlen)

        self.L = threading.Lock()
        self.active = True
        
        pn = threading.Thread(target=self.getNet).start()

    def getNet(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.IP, self.PORT))
        s.setblocking(1)
        
        while True:
            with self.L:
                if not self.active: break

            data = s.recv(8192)
            a = str(data).split(' ')
            if self.v: print a
            if a[1] == '0':
                self.cartesian.appendleft([a[2:5], a[5:]])
            elif a[1] == '1':
                self.joints.appendleft([a[2:5], a[5:]])

        #supposedly not necessary but the robot gets mad if you don't do this
        s.shutdown(socket.SHUT_RDWR)
        s.close()
        

    def writeData(self, jfilename='joints.txt', cfilename='cartesian.txt'):
        fj = open(jfilename, 'w')
        for j in self.joints:
            for i in j[0]: fj.write(i+ ' ')
            for i in j[1]: fj.write(i+ ' ')
            fj.write('\n')
        fj.close()

        fc = open(cfilename, 'w')
        for c in self.cartesian:
            for i in c[0]: fc.write(i + ' ')
            for i in c[1]: fc.write(i + ' ')
            fc.write('\n')
        fc.close()

    def close(self):
        self.stop()

    def stop(self):
        with self.L:
            self.active=False

    def __del__(self): 
        self.stop()
