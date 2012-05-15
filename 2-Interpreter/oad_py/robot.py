'''
Michael Dawson-Haggerty
CMU Robotics Institute
For David Bourne/iFAB


robot.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code modules LOGGER and SERVER)
'''

import socket, json, os
    
class Robot:
    def __init__(self, IP='192.168.125.1', PORT=5000, wobj=[[0,0,0],[1,0,0,0]], tool=[[0,0,0], [1,0,0,0]], speed = [100,50], toolfile=None, zeroJoints = False, verbose=False):
        
        self.BUFLEN = 4096 
        self.v = verbose

        if verbose: print 'Attempting to connect to ABB robot at', IP
        try: 
            self.robsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robsock.connect((IP, PORT))
            if verbose: print 'Connected to robot successfully.'
        except:
            print 'Connection to robot failed!'
        
        if toolfile == None: self.setTool(tool)
        else: setToolFile(toolfile)
        self.setWorkObject(wobj)
        self.setSpeed(speed)
        self.setZone('z5')
        if zeroJoints: self.setJoints()

    def setCartesian(self, pos):
        if len(pos) == 7: pos = [pos[0:3], pos[3:7]]
        if self.checkCoordinates(pos):
            msg = "01 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            if self.v: print 'setCart:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else:
            return False

    def setTool(self, tool=[[0,0,0], [1,0,0,0]]):
        #sets the tool object of the robot. 
        #Offsets are from tool0 (tool flange center axis/ flange face intersection)
        if len(tool) == 7: tool = [tool[0:3], tool[3:7]]
        if self.checkCoordinates(tool):
            msg = "06 000 " 
            msg = msg + format(tool[0][0], "+08.1f") + " " + format(tool[0][1], "+08.1f") + " " + format(tool[0][2], "+08.1f") + " " 
            msg = msg + format(tool[1][0], "+08.5f") + " " + format(tool[1][1], "+08.5f") + " " 
            msg = msg + format(tool[1][2], "+08.5f") + " " + format(tool[1][3], "+08.5f") + " #"    
            if self.v: print 'setTool:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            self.tool = tool
            return data
        else: return False

    def setWorkObject(self, wobj=[[0,0,0],[1,0,0,0]]):
        #wobj is [[cartesians], [quaternions]] or [cartesians, quaternions]
        if len(wobj) == 7: pos = [wobj[0:3], wobj[3:7]]
        if self.checkCoordinates(wobj):
            msg = "07 " 
            msg = msg + format(wobj[0][0], "+08.1f") + " " + format(wobj[0][1], "+08.1f") + " " + format(wobj[0][2], "+08.1f") + " " 
            msg = msg + format(wobj[1][0], "+08.5f") + " " + format(wobj[1][1], "+08.5f") + " " 
            msg = msg + format(wobj[1][2], "+08.5f") + " " + format(wobj[1][3], "+08.5f") + " #"    
            if self.v: print 'setWorkObject:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else: return False
            
    #speed is [linear speed (mm/s), orientation speed (deg/s)]. 
    def setSpeed(self, speed=[100,50]):
        if len(speed) == 2:
            msg = "08 " 
            msg = msg + format(speed[0], "+08.1f") + " " + format(speed[1], "+08.2f") + " #"  
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            if self.v: print 'setSpeed:', msg 
            return data
        else: return False


    def circularSingle(self, circlePoint, endPoint):
        
        if len(circlePoint) == 7: circlePoint = [circlePoint[0:3], circlePoint[3:7]]
        if len(endPoint) == 7: endPoint = [endPoint[0:3], endPoint[3:7]]

        if self.checkCoordinates(circlePoint) & self.checkCoordinates(endPoint):
            msg = "35 000 " 
            msg = msg + format(circlePoint[0][0], "+08.1f") + " " + format(circlePoint[0][1], "+08.1f") + " " + format(circlePoint[0][2], "+08.1f") + " " 
            msg = msg + format(circlePoint[1][0], "+08.5f") + " " + format(circlePoint[1][1], "+08.5f") + " " 
            msg = msg + format(circlePoint[1][2], "+08.5f") + " " + format(circlePoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)

            msg = "36 " 
            msg = msg + format(endPoint[0][0], "+08.1f") + " " + format(endPoint[0][1], "+08.1f") + " " + format(endPoint[0][2], "+08.1f") + " " 
            msg = msg + format(endPoint[1][0], "+08.5f") + " " + format(endPoint[1][1], "+08.5f") + " " 
            msg = msg + format(endPoint[1][2], "+08.5f") + " " + format(endPoint[1][3], "+08.5f") + " #"    

            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            return data
        else:
            print 'coordinate check failed'
            return False
    
    
    def setExternalAxis(self, height):
        #our table goes from -550 to +40
        msg = "45 "
        msg = msg + format(height, "+08.1f") + " #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

    def setBuffer(self, posList, gotoFirst=False):
        #adds every position in posList to the buffer
        
        self.clearBuffer()
        if self.lenBuffer() <> 0: return False

        for i in posList: self.addBuffer(i)
        if gotoFirst: self.setCartesian(posList[0])
        if self.lenBuffer() == len(posList): return True
        else: 
            self.clearBuffer()
            return False


    def addBuffer(self, pos):
        #appends single target to the buffer
        if len(pos) == 7: pos = [pos[0:3], pos[3:7]]
        if self.checkCoordinates(pos):
            msg = "10 " 
            msg = msg + format(pos[0][0], "+08.1f") + " " + format(pos[0][1], "+08.1f") + " " + format(pos[0][2], "+08.1f") + " " 
            msg = msg + format(pos[1][0], "+08.5f") + " " + format(pos[1][1], "+08.5f") + " " 
            msg = msg + format(pos[1][2], "+08.5f") + " " + format(pos[1][3], "+08.5f") + " #"    
            if self.v: print 'addBuffer:', msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            #time.sleep(.1)
            return data
        else:
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


    def executeBuffer(self):
        #execute every move in buffer as MoveL command (linear move)
        msg = "13 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data

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

    def getJoints(self):
        msg = "04 #"
        self.robsock.send(msg)
        data = numpy.float_(str(self.robsock.recv(self.BUFLEN)).split(' '))
        return data[4:10]

    def getCartesian(self):
        msg = "03 #"
        self.robsock.send(msg)
        data = numpy.float_(str(self.robsock.recv(self.BUFLEN)).split(' '))
        return [data[4:7], data[7:11]]
        
    def setZone(self, zoneKey='z1', finep = False, manualZone=[]):
        zoneDict = {'z0': [.3,.3,.03], 'z1': [1,1,.1], 'z5': [5,8,.8], 'z10': [10,15,1.5], 'z15': [15,23,2.3], 'z20': [20,30,3], 'z30': [30,45,4.5], 'z50': [50,75,7.5], 'z100': [100,150,15], 'z200': [200,300,30]}

        #zoneKey: uses values from RAPID handbook (stored here in zoneDict), 'z*' YOU SHOULD USE THESE
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
        if self.v: print 'setZone key', zoneKey
        return data

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
        self.close()
