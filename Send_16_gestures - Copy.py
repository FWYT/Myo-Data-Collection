
import myo
import math
import time
import numpy
import numpy.matlib
import socket
myo.init()

from myo.six import print_

def convert_to_rpy(qternion):
    x = qternion[0]
    y = qternion[1]
    z = qternion[2]
    w = qternion[3]

    roll1=math.atan2(2.0*(w*x + y*z),1.0 - 2.0*(x**2+y**2))
    pitch1=math.asin(max(-1.0,min(1.0, 2.0*(w*y-z*x))))
    yaw1=math.atan2(2.0*(w*z + x*y),1.0 - 2.0*(y**2 + z**2))

    roll1=math.degrees(roll1)
    pitch1=math.degrees(pitch1)
    yaw1=math.degrees(yaw1)
    output=[roll1,pitch1,yaw1]
    return output

def assign_pose_number(pose):
    global pose_number
    if (pose == 'rest'):
        pose_number = 0
        pose_name = 'rest'

    elif (pose=='fingers_spread'):
        pose_number = 1
        pose_name ='fingers_spread'

    elif (pose=='doubleTap'):
        pose_number = 2
        pose_name ='doubleTap'

    elif (pose=='fist'):
        pose_number = 3
        pose_name = 'fist'

    elif (pose=='wave_in'):
        pose_number = 4
        pose_name = 'wave_in'

    elif (pose=='wave_out'):
        pose_number = 5
        pose_name = 'wave_out'

    return pose_number


class Listener(myo.DeviceListener):
    #return False from any method to stop the Hub

    def __init__(self,address,port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((address, port))
        self._sock.setblocking(False)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.enable = 0
        self.odr=0
        self.counter2=0
        self.rcfull=[None]*50
        self.rsfull=[None]*50
        self.pcfull=[None]*50
        self.psfull=[None]*50
        self.ycfull=[None]*50
        self.ysfull=[None]*50
        self.rollb=0
        self.pitchb=0
        self.yawb=0
        self.state=-1
        self.fistcount=0
        self.W=[]


    def on_connect(self, myo, timestamp):
        print_("Connected to Myo")
        myo.vibrate('short')
        myo.request_rssi()

    def on_rssi(self, myo, timestamp, rssi):
        print_("RSSI:", rssi)

    def on_event(self, event):
        r""" Called before any of the event callbacks. """

    def on_event_finished(self, event):
        r""" Called after the respective event callbacks have been
        invoked. This method is *always* triggered, even if one of
        the callbacks requested the stop of the Hub. """

    def on_pair(self, myo, timestamp):
        print_('on_pair')

    def on_disconnect(self, myo, timestamp):
        print_('on_disconnect')

    def on_pose(self, myo, timestamp, pose):
        self.nowpose=pose
        print_('on_pose', pose)
        if pose == "fist":
            self.fistcount=self.fistcount+1
            if self.fistcount==1:
                self.state=0
                print("Entering state 0")
        



    def on_orientation_data(self, myo, timestamp, orientation):
        self.odr=orientation
        if self.state==-1:
            calib_start=input('Press 1 for calibration: ')
            self.state=0
            print("Beginning Calibration!!!")


        if self.counter2<50:
            output=convert_to_rpy(self.odr)
            roll1=output[0]
            pitch1=output[1]
            yaw1=output[2]
            #print_(roll1,pitch1,yaw1)

            self.rcfull[self.counter2] = math.cos(math.radians(roll1))
            self.rsfull[self.counter2] = math.sin(math.radians(roll1))
            self.pcfull[self.counter2] = math.cos(math.radians(pitch1))
            self.psfull[self.counter2] = math.sin(math.radians(pitch1))
            self.ycfull[self.counter2] = math.cos(math.radians(yaw1))
            self.ysfull[self.counter2] = math.sin(math.radians(yaw1))
        self.counter2=self.counter2+1
       
        
        

        if self.counter2 == 50:
            rc = numpy.mean(self.rcfull)
            pc = numpy.mean(self.pcfull)
            yc = numpy.mean(self.ycfull)
            rs = numpy.mean(self.rsfull)
            ps = numpy.mean(self.psfull)
            ys = numpy.mean(self.ysfull)

            if self.state==0:  # calibration

                self.rollb = math.degrees(math.atan2(rs,rc))
                self.pitchb = math.degrees(math.atan2(ps,pc))
                self.yawb = math.degrees(math.atan2(ys,yc))
                baseline=[self.rollb,self.pitchb,self.yawb]
                print(baseline)
                print("Calibration Complete! You have 2 seconds to change your gesture and then record a new one for 2")
                self.counter2=0
                self.state=1
                self.start_time = time.time()

            elif self.state == 1:

                #for i in range(1,1000):
                #    for j in range(1,10000):
                #        a=i
                time.sleep(2)
                self.counter2 = 0
                self.state = 2

            elif self.state == 2:  # data collection, identification and transfer

                self.roll = math.degrees(math.atan2(rs,rc))-self.rollb
                self.pitch = math.degrees(math.atan2(ps,pc))-self.pitchb
                self.yaw = math.degrees(math.atan2(ys,yc))-self.yawb
                a=numpy.matrix([self.roll,self.pitch,self.yaw])
                b=a.transpose()
             
                d=numpy.matlib.repmat(b,1,4) #hardcoded------CHANGE LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              
                pose_number=assign_pose_number(self.nowpose)
                self.W=numpy.matrix([[-2.38946624937314,172.907592400722,-12.1921835866013,30.7345074055692],[-1.60615657009811, 70.7802307763561,-10.0331866141660,-71.1455632604777],[5.87052357307342,52.4896992580217,-45.3268722289868,-67.7618396439250]])
                diff=self.W-d
              
                squared=numpy.square(diff)
              
                summed=sum(squared)
             
                #rooted=numpy.sqrt(summed)
                #rooted=rooted.tolist()
                rooted=summed.tolist()
                print("rooted", rooted)
               

                P = min(rooted[0])
                #print ("P ", P)
                Q = rooted[0].index(P)
                #print ("Q", Q)
                R = Q + 1
                #print("R" , R)




                #indx=rooted.index(min(rooted))+1#1-front; 2-up; 3-side; 4-down
                indx=R
                print ("indx",indx)
                print ("pose num ", pose_number)

                if indx==1:#front
                    if pose_number==1:
                        gesture_number=1
                        gesture_name='FRONT_finger_spread'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 1\n", ("<broadcast>", 10000))
                    elif pose_number==3:
                        gesture_number=2
                        gesture_name='FRONT_fist'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 3\n", ("<broadcast>", 10000))
                    elif pose_number==4:
                        gesture_number=3
                        gesture_name='FRONT_wave_in'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 4\n", ("<broadcast>", 10000))
                    elif pose_number==5:
                        gesture_number=4
                        gesture_name='FRONT_wave_out'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 5\n", ("<broadcast>", 10000))

                elif indx==2:#top
                    if pose_number==1:
                        gesture_number=5
                        gesture_name='UP_finger_spread'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 101\n", ("<broadcast>", 10000))
                    elif pose_number==3:
                        gesture_number=6
                        gesture_name='UP_fist'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 103\n", ("<broadcast>", 10000))
                    elif pose_number==4:
                        gesture_number=7
                        gesture_name='UP_wave_in'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 104\n", ("<broadcast>", 10000))
                    elif pose_number==5:
                        gesture_number=8
                        gesture_name='UP_wave_out'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 105\n", ("<broadcast>", 10000))

                elif indx==3:#side
                    if pose_number==1:
                        gesture_number=9
                        gesture_name='SIDE_finger_spread'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 201\n", ("<broadcast>", 10000))
                    elif pose_number==3:
                        gesture_number=10
                        gesture_name='SIDE__fist'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 203\n", ("<broadcast>", 10000))
                    elif pose_number==4:
                        gesture_number=11
                        gesture_name='SIDE__wave_in'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 204\n", ("<broadcast>", 10000))
                    elif pose_number==5:
                        gesture_number=12
                        gesture_name='SIDE__wave_out'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 205\n", ("<broadcast>", 10000))

                elif indx==4:#down
                    if pose_number==1:
                        gesture_number=13
                        gesture_name='DOWN_finger_spread'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 301\n", ("<broadcast>", 10000))
                    elif pose_number==3:
                        gesture_number=14
                        gesture_name='DOWN__fist'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 303\n", ("<broadcast>", 10000))
                    elif pose_number==4:
                        gesture_number=15
                        gesture_name='DOWN__wave_in'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 304\n", ("<broadcast>", 10000))
                    elif pose_number==5:
                        gesture_number=16
                        gesture_name='DOWN__wave_out'
                        print_(gesture_name)
                        self._sock.sendto("Gesture: 305\n", ("<broadcast>", 10000))

                #print(pose_number,self.roll,self.pitch,self.yaw)

                self.counter2=0
                self.state=2


            #print_(orientation)


    def on_accelerometor_data(self, myo, timestamp, acceleration):
        pass

    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        pass

def main():
    #address = "192.168.1.110"
    address = "localhost"
    port = 0
    hub = myo.Hub()
    hub.run(10000000, Listener(address, port))
    # Listen to keyboard interrupts and stop the
    # hub in that case.
    try:
        while hub.running:
            myo.time.sleep(0.2)
    except KeyboardInterrupt:
        print_("Quitting ...")
        hub.stop(True)

if __name__ == '__main__':
    main()

