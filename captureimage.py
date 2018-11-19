# coding:utf-8
__author__ = 'mr tang'
__date__ = '2014-09-02'

import time
import cv2 as cv
import multiprocessing
from multiprocessing import Queue, Event
from Basebyd import *
import socket

def test():
    Server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    Server.bind(('',bydparams['PORT_computer']))
    time.sleep(1)
    bydstate = BydState()
    while True:
        BUF,(addr,port) = Server.recvfrom(1024)
        if bydstate.Parsing(BUF):
            print bydstate.SteeringAngle
            print bydstate.LeftLamp
            print bydstate.RightLamp
            print bydstate.EStop
    Server.close()


    
def subprocess(que,ev):
    Server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    Server.bind(('',bydparams['PORT_computer']))
    time.sleep(1)
    bydstate = BydState()
    while True:
        if ev.is_set():break
        BUF,(addr,port) = Server.recvfrom(1024)
        if bydstate.Parsing(BUF):
            que.put(bydstate.SteeringAngle)
            # print bydstate.SteeringAngle
    Server.close()
    
def subprocess1(que,ev):
    i = 0
    time.sleep(1)
    while True:
        if ev.is_set():break
        i += 1
        que.put(i)
        time.sleep(0.1)
    

class SynCapImageSteer:
    def __init__(self):
        self.cap = cv.VideoCapture(0)
        with open('byddataset//data.txt','w') as f:
            pass 
        self.f = open('byddataset//data.txt','a')

    def main(self):
        que = Queue()
        ev = Event()
        print 'yes'
        pro = multiprocessing.Process(target = subprocess, args = (que,ev))
        pro.start()
        i = 0
        print '????'
        while(cv.waitKey(1) != ord('q')):
        # while True:
            a = que.get()
            print a
            ret, frame = self.cap.read()
            cv.imshow('frame',frame)
            i += 1
            cv.imwrite('byddataset//%s.jpg'%i,frame)
            self.f.write('%s.jpg    %s\n'%(i,a))
            
        self.cap.release()
        cv.destroyAllWindows()
        ev.set()
        self.f.close()
        time.sleep(1)
        
        
def testsubprocess():
    que = Queue()
    ev = Event()
    subprocess(que,ev)

if __name__ == '__main__':
    # s = SynCapImageSteer()
    # s.main()
    # testsubprocess()
    test()
